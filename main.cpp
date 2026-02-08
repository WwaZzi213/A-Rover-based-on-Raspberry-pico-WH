// wireless control Version 1 test version 2026.2.8 13:10 chatgpt 




#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"   // WiFi + LED (Pico W)

#include <cstdio>
#include <cstring>
#include <cstdint>

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

#include "dhcpserver.h"

// -------------------------
// Motor driver pin mapping
// -------------------------
constexpr uint M1_PWM_GPIO = 2;   // Left motor PWM  (GP2 -> PWM1)
constexpr uint M1_DIR_GPIO = 3;   // Left motor DIR  (GP3 -> DIR1)
constexpr uint M2_PWM_GPIO = 0;   // Right motor PWM (GP0 -> PWM2)
constexpr uint M2_DIR_GPIO = 5;   // Right motor DIR (GP5 -> DIR2)

constexpr float PWM_FREQ_HZ = 20000.0f;  // 20 kHz PWM (quiet)

// Fixed speeds (percentage, -100 .. 100)
constexpr int   DRIVE_SPEED  = 50;
constexpr int   TURN_SPEED   = 50;
constexpr float INNER_FACTOR = 0.4f;

// =====================================================
// Motor: basic speed + direction control
// =====================================================
class Motor {
public:
    Motor(uint pwm_gpio, uint dir_gpio)
        : pwm_gpio_(pwm_gpio), dir_gpio_(dir_gpio) {}

    void init() {
        gpio_init(dir_gpio_);
        gpio_set_dir(dir_gpio_, GPIO_OUT);
        gpio_put(dir_gpio_, 0);

        gpio_set_function(pwm_gpio_, GPIO_FUNC_PWM);

        uint slice = pwm_gpio_to_slice_num(pwm_gpio_);
        uint chan  = pwm_gpio_to_channel(pwm_gpio_);

        float div = (float)clock_get_hz(clk_sys) / (PWM_FREQ_HZ * 65536.0f);
        if (div < 1.0f) div = 1.0f;
        if (div > 255.0f) div = 255.0f;

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, div);
        pwm_config_set_wrap(&cfg, 65535);
        pwm_init(slice, &cfg, true);

        pwm_set_chan_level(slice, chan, 0);
    }

    void setSpeed(int speed_percent) {
        if (speed_percent > 100) speed_percent = 100;
        if (speed_percent < -100) speed_percent = -100;

        bool reverse = (speed_percent < 0);
        int mag = reverse ? -speed_percent : speed_percent;

        gpio_put(dir_gpio_, reverse ? 1 : 0);

        uint slice = pwm_gpio_to_slice_num(pwm_gpio_);
        uint chan  = pwm_gpio_to_channel(pwm_gpio_);

        uint16_t level = static_cast<uint16_t>((mag * 65535) / 100);
        pwm_set_chan_level(slice, chan, level);
    }

private:
    uint pwm_gpio_;
    uint dir_gpio_;
};

// =====================================================
// Car: differential drive abstraction
// =====================================================
class Car {
public:
    Car(uint left_pwm, uint left_dir, uint right_pwm, uint right_dir)
        : left_(left_pwm, left_dir),
          right_(right_pwm, right_dir) {}

    void init() {
        left_.init();
        right_.init();
        stop();
    }

    void stop() {
        left_.setSpeed(0);
        right_.setSpeed(0);
    }

    void forward() {
        left_.setSpeed( LEFT_SIGN  * DRIVE_SPEED);
        right_.setSpeed(RIGHT_SIGN * DRIVE_SPEED);
    }

    void backward() {
        left_.setSpeed( LEFT_SIGN  * -DRIVE_SPEED);
        right_.setSpeed(RIGHT_SIGN * -DRIVE_SPEED);
    }

    void turnLeft() {
        int inner = static_cast<int>(TURN_SPEED * INNER_FACTOR);
        int outer = TURN_SPEED;
        left_.setSpeed( LEFT_SIGN  * inner);
        right_.setSpeed(RIGHT_SIGN * outer);
    }

    void turnRight() {
        int inner = static_cast<int>(TURN_SPEED * INNER_FACTOR);
        int outer = TURN_SPEED;
        left_.setSpeed( LEFT_SIGN  * outer);
        right_.setSpeed(RIGHT_SIGN * inner);
    }

private:
    static constexpr int LEFT_SIGN  = +1;
    static constexpr int RIGHT_SIGN = -1;

    Motor left_;
    Motor right_;
};

// =========================
// WiFi AP + HTTP control
// =========================
static const char* WIFI_SSID = "ROVER_CTRL";
static const char* WIFI_PASS = "12345678"; // 至少8位；如不想密码可改成 OPEN(见下)

enum class Cmd { Stop, Forward, Backward, Left, Right };

static volatile Cmd g_cmd = Cmd::Stop;
static absolute_time_t g_last_cmd_time;

static const int CMD_TIMEOUT_MS = 350; // 超时自动停车（手动遥控必做）

static const char INDEX_HTML[] =
R"HTML(<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ROVER CTRL</title>
<style>
body{font-family:system-ui;margin:16px}
h2{margin:0 0 12px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;max-width:420px}
.btn{padding:18px 10px;font-size:18px;border:1px solid #333;border-radius:12px;background:#f2f2f2}
.btn:active{background:#ddd}
.small{font-size:14px;opacity:.7;margin-top:12px}
</style>
</head>
<body>
<h2>ROVER CTRL</h2>
<div class="grid">
  <div></div>
  <button class="btn" id="f">▲</button>
  <div></div>

  <button class="btn" id="l">◀</button>
  <button class="btn" id="s">■ STOP</button>
  <button class="btn" id="r">▶</button>

  <div></div>
  <button class="btn" id="b">▼</button>
  <div></div>
</div>
<div class="small">
连接到热点 <b>ROVER_CTRL</b> 后打开：<b>http://192.168.4.1/</b><br/>
按住方向键运动，松开停止。
</div>

<script>
function send(d){ fetch('/cmd?d='+d).catch(()=>{}); }
function hold(btn, d){
  let down=false;
  const onDown=(e)=>{ e.preventDefault(); down=true; send(d); };
  const onUp=(e)=>{ e.preventDefault(); down=false; send('S'); };
  btn.addEventListener('mousedown', onDown);
  btn.addEventListener('mouseup', onUp);
  btn.addEventListener('mouseleave', onUp);
  btn.addEventListener('touchstart', onDown, {passive:false});
  btn.addEventListener('touchend', onUp, {passive:false});
  btn.addEventListener('touchcancel', onUp, {passive:false});
}
hold(document.getElementById('f'),'F');
hold(document.getElementById('b'),'B');
hold(document.getElementById('l'),'L');
hold(document.getElementById('r'),'R');
document.getElementById('s').onclick=()=>send('S');
</script>
</body>
</html>)HTML";

static void apply_cmd_char(char c) {
    switch (c) {
        case 'F': g_cmd = Cmd::Forward;  break;
        case 'B': g_cmd = Cmd::Backward; break;
        case 'L': g_cmd = Cmd::Left;     break;
        case 'R': g_cmd = Cmd::Right;    break;
        default:  g_cmd = Cmd::Stop;     break;
    }
    g_last_cmd_time = get_absolute_time();
}

// Very small HTTP parser: only GET / and GET /cmd?d=X
static void handle_http_request(int client_fd) {
    char buf[1024];
    int n = lwip_read(client_fd, buf, (int)sizeof(buf) - 1);
    if (n <= 0) return;
    buf[n] = '\0';

    // Parse first line: "GET /path HTTP/1.1"
    // We only care about the path.
    const char* p = strstr(buf, "GET ");
    if (!p) return;
    p += 4;
    const char* sp = strchr(p, ' ');
    if (!sp) return;

    char path[256];
    size_t path_len = (size_t)(sp - p);
    if (path_len >= sizeof(path)) path_len = sizeof(path) - 1;
    memcpy(path, p, path_len);
    path[path_len] = '\0';

    if (strncmp(path, "/cmd", 4) == 0) {
        // Find d=?
        const char* q = strstr(path, "d=");
        if (q && q[2]) {
            apply_cmd_char(q[2]);
        }

        const char* resp =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain\r\n"
            "Cache-Control: no-store\r\n"
            "Connection: close\r\n\r\n"
            "OK\n";
        lwip_write(client_fd, resp, strlen(resp));
        return;
    }

    // Default: serve page
    char header[256];
    int hl = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n"
        "Content-Length: %u\r\n\r\n",
        (unsigned)strlen(INDEX_HTML)
    );
    lwip_write(client_fd, header, hl);
    lwip_write(client_fd, INDEX_HTML, strlen(INDEX_HTML));
}

static int start_http_server_socket() {
    int fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    int yes = 1;
    lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = PP_HTONS(80);
    addr.sin_addr.s_addr = PP_HTONL(INADDR_ANY);

    if (lwip_bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        lwip_close(fd);
        return -1;
    }

    if (lwip_listen(fd, 4) != 0) {
        lwip_close(fd);
        return -1;
    }

    // Non-blocking accept
    int flags = lwip_fcntl(fd, F_GETFL, 0);
    lwip_fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    return fd;
}

// =====================================================
// Main
// =====================================================
int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        while (true) { tight_loop_contents(); }
    }

    // Blink LED to show boot OK
    for (int i = 0; i < 3; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(150);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(150);
    }

    Car car(M1_PWM_GPIO, M1_DIR_GPIO, M2_PWM_GPIO, M2_DIR_GPIO);
    car.init();

    // ---- Start AP mode ----
    // WPA2: CYW43_AUTH_WPA2_AES_PSK
    // 如果你想开放热点无密码：把 WIFI_PASS 设为 "" 并把 auth 改成 CYW43_AUTH_OPEN
    cyw43_arch_enable_ap_mode(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK);

    // Set AP interface IP: 192.168.4.1/24
    ip4_addr_t ip, mask, gw;
    IP4_ADDR(&ip,   192, 168, 4, 1);
    IP4_ADDR(&mask, 255, 255, 255, 0);
    IP4_ADDR(&gw,   192, 168, 4, 1);

    // cyw43_state.netif[CYW43_ITF_AP] is the AP netif
    extern cyw43_t cyw43_state;
    struct netif* ap_netif = &cyw43_state.netif[CYW43_ITF_AP];

    cyw43_arch_lwip_begin();
    netif_set_addr(ap_netif, &ip, &mask, &gw);
    netif_set_up(ap_netif);
    cyw43_arch_lwip_end();

    // Start DHCP server so phone gets an IP automatically
    dhcp_server_t* dhcp = nullptr;
    cyw43_arch_lwip_begin();
    dhcp = dhcp_server_start(ap_netif, &ip, &mask, &gw);
    cyw43_arch_lwip_end();

    // Start HTTP server (socket)
    cyw43_arch_lwip_begin();
    int listen_fd = start_http_server_socket();
    cyw43_arch_lwip_end();

    g_last_cmd_time = get_absolute_time();

    // Main loop: accept HTTP, apply cmd, drive car, timeout stop
    while (true) {
        // Accept at most one client per tick (enough for this use)
        cyw43_arch_lwip_begin();
        struct sockaddr_in client_addr {};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = lwip_accept(listen_fd, (struct sockaddr*)&client_addr, &client_len);
        cyw43_arch_lwip_end();

        if (client_fd >= 0) {
            // Handle quickly then close
            cyw43_arch_lwip_begin();
            handle_http_request(client_fd);
            lwip_close(client_fd);
            cyw43_arch_lwip_end();
        }

        // Drive based on latest command
        // Timeout protection: no new cmd => stop
        if (absolute_time_diff_us(g_last_cmd_time, get_absolute_time()) > (CMD_TIMEOUT_MS * 1000)) {
            g_cmd = Cmd::Stop;
        }

        switch (g_cmd) {
            case Cmd::Forward:  car.forward();  break;
            case Cmd::Backward: car.backward(); break;
            case Cmd::Left:     car.turnLeft(); break;
            case Cmd::Right:    car.turnRight();break;
            default:            car.stop();     break;
        }

        sleep_ms(10);
    }

    // (never reached)
    if (dhcp) dhcp_server_stop(dhcp);
    cyw43_arch_deinit();
    return 0;
}
