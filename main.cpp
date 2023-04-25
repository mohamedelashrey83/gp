#include <WiFi.h>
#include <WebServer.h>
/*.......................................*/
const char *ssid = "mohamed";
const char *pass = "12345678";
IPAddress local_ip(192, 168, 2, 1);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);
bool VRC_status = false;
typedef enum
{
    forward,
    right,
    left,
    back
} dir;
dir d = forward;
void HandleOnConnect();
void handle_on();
void handle_off();
void handle_right();
void handle_left();
void handle_forward();
void handle_back();
void handle_charge();
String send_html(bool s, dir x);

/*.......................................*/
void setup()
{
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    WiFi.softAP(ssid, pass);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(100);
    server.on("/", HandleOnConnect);
    server.on("/on", handle_on);
    server.on("/off", handle_off);
    server.on("/right", handle_right);
    server.on("/left", handle_left);
    server.on("/forward", handle_forward);
    server.on("/back", handle_back);
    server.begin();
}
void loop()
{
    server.handleClient();
    if (VRC_status)
    {
        digitalWrite(2, HIGH);

        // if (d == forward)
        // {
        //     digitalWrite(2, HIGH);
        // }
        if (d == right)
        {
            digitalWrite(3, HIGH);
        }
        else if (d == left)
        {
            digitalWrite(4, LOW);
        }
        else if (d == back)
        {
            digitalWrite(5, HIGH);
        }
    }
    else if (VRC_status == false)
    {
        digitalWrite(2, LOW);
    }
}
void HandleOnConnect()
{
    VRC_status = LOW;
    server.send(200, "text/html", send_html(VRC_status, forward));
}

void handle_on()
{
    VRC_status = true;
    server.send(200, "text/html", send_html(true, forward));
}
void handle_off()
{
    VRC_status = false;
    server.send(200, "text/html", send_html(false, forward));
}
void handle_right()
{
    d = right;
    server.send(200, "text/html", send_html(true, right));
}
void handle_left()
{
    d = left;
    server.send(200, "text/html", send_html(true, left));
}
void handle_forward()
{
    d = forward;
    server.send(200, "text/html", send_html(true, forward));
}
void handle_back()
{
    d = back;
    server.send(200, "text/html", send_html(true, back));
}
String send_html(bool s, dir x)
{
    String ptr = "<!DOCTYPE html> <html>\n";
    ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
    ptr += "<title>ROOMBA</title>\n";
    /*.........................style.............................*/
    ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
    ptr += "body{margin-top: 50px;}";
    ptr += "h1 {color: #4717d7;margin: 50px auto 30px;font-family: 'Times New Roman', Times, serif;}\n";
    ptr += "h3{color: black;margin: auto;}";
    ptr += ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
    ptr += ".button-on {background-color: #3498db;}\n";
    ptr += ".button-on:active {background-color: #2980b9;}\n";
    ptr += ".button-off {background-color: #34495e;}\n";
    ptr += ".button-off:active {background-color: #2c3e50;}\n";
    ptr += "p {font-size: 16px;color: #911515;margin-bottom: 10px;}\n";
    ptr += ".container {display: flex;justify-content: baseline;align-items: center;height: 20vh;}";
    ptr += "button {background-color: rgba(22, 213, 22, 0.6);border: #000;color: #000;padding: 12px 24px;text-align: center;text-decoration: none;display: inline-block;font-size: 50px;margin: 10px 10px;cursor: pointer;}\n";
    ptr += "#battery {width: 300px;height: 50px;border: 3px solid #100;position: relative;background-color: #f5f5f5;border-radius: 20px;overflow: hidden;}\n";
    ptr += "#battery-level {height: 100%;position: absolute;top: 0;left: 0;background-color: #f61;}";
    ptr += "#percentage {font - size : 24px;margin - top : 10px;}";
    ptr += "</style>\n";
    // ptr+="";
    /*...............................................................*/
    ptr += "</head>\n";
    ptr += "<h1>ROOMBA.....Welcome</h1>";

    ptr += "<h3>Control of Vacuum Robot Cleaer</h3>";
    ptr += "<h1>Robot Battery Percentage</h1>";
    ptr += "<div id=\"battery\"><div id=\"battery-level\" style=\"width: 40%;\"></div></div>";
    ptr += "<div id=\"percentage\">40%</div>";
    if (s == true)
    {
        if (x == forward)
        {
            ptr += "<p>Robot Status:   on</p>";
            ptr += "<a class=\"button button-off\" href=\"/off\">OFF</a>";
            // ptr += "<a class=\"button button-on\" href=\"/right\">RIGHT</a>";
            // ptr += "<a class=\"button button-on\" href=\"/left\">LEFT</a>";
            ptr += "<a href=\"/forward\"><button class=\"up\">&#8593;</button></a>";
            ptr += "<div><a href=\"/left\"><button class=\"left\">&#8592;</button></a><a href=\"/right\"><button class=\"right\">&#8594;</button></a></div>";
            ptr += "<a href=\"/back\"><button class=\"down\">&#8595;</button></a>";
        }
        else if (x == left)
        {
            ptr += "<p>Robot Status:   LEFT</p>";
            ptr += "<a class=\"button button-off\" href=\"/off\">OFF</a>";
            // ptr += "<a class=\"button button-on\" href=\"/right\">RIGHT</a>";
            // ptr += "<a class=\"button button-on\" href=\"/left\">LEFT</a>";
            ptr += "<a href=\"/forward\"><button class=\"up\">&#8593;</button></a>";
            ptr += "<div><a href=\"/left\"><button class=\"left\">&#8592;</button></a><a href=\"/right\"><button class=\"right\">&#8594;</button></a></div>";
            ptr += "<a href=\"/back\"><button class=\"down\">&#8595;</button></a>";
        }
        else if (x == right)
        {
            ptr += "<p>Robot Status:   RIGHT</p>";
            ptr += "<a class=\"button button-off\" href=\"/off\">OFF</a>";
            // ptr += "<a class=\"button button-on\" href=\"/right\">RIGHT</a>";
            // ptr += "<a class=\"button button-on\" href=\"/left\">LEFT</a>";
            ptr += "<a href=\"/forward\"><button class=\"up\">&#8593;</button></a>";
            ptr += "<div><a href=\"/left\"><button class=\"left\">&#8592;</button></a><a href=\"/right\"><button class=\"right\">&#8594;</button></a></div>";
            ptr += "<a href=\"/back\"><button class=\"down\">&#8595;</button></a>";
        }
        else if (x == back)
        {
        }
    }
    else
    {
        ptr += "<p>Robot Status:off</p>";
        ptr += "<a class=\"button button-on\" href=\"/on\">ON</a>";
    }

    // ptr+="";
    // ptr+="";
    ptr += "</body>\n";
    ptr += "</html>\n";
    return ptr;
}