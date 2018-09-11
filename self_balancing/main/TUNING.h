const static char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";
const static char http_index_hml[] = "<html><head><title></title></head><body><table border = 1; cellspacing=0; width=70%><tr><td bgcolor = #9A9594; height = 40px; colspan=2 ; align=center ><b><i>TUNING</i></b></td></tr><tr><td height=25px; bgcolor=#DAD7D7> <a href =\"a\">Increase Pitch Kp</a></td><td height=25px; bgcolor=#DAD7D7> <a href =\"b\">Decrease Pitch Kp</a> </td></tr><tr><td height=25px><a href =\"c\">Increase Pitch Kd </a></td><td height=25px> <a href =\"d\">Decrease Pitch Kd </a></td></tr><tr><td height=25px; bgcolor=#DAD7D7> <a href =\"e\">Increase Pitch Ki </a></td><td height=25px; bgcolor=#DAD7D7> <a href =\"g\"> Decrease Pitch Ki </a></td></tr><tr><td height=25px> <a href =\"h\">Increase Yaw Kp </a></td><td height=25px> <a href =\"i\">Decrease Yaw Kp </a></td></tr><tr><td height=25px; bgcolor=#DAD7D7> <a href =\"j\">Increase Yaw Kd </a></td><td height=25px; bgcolor=#DAD7D7> <a href =\"k\">Decrease Yaw Kd </a></td></tr> <tr><td height=25px> <a href =\"l\">Increase Yaw Ki </a></td><td height=25px> <a href =\"m\">Decrease Yaw Ki </a></td></tr> <tr><td height=25px; bgcolor=#DAD7D7><a href =\"n\">Increase MAX_PWM </a></td><td height=25px; bgcolor=#DAD7D7><a href =\"o\">Decrease MAX_PWM</a></td></tr><tr><td height=25px> <a href =\"p\">Increase Min Pwm</a></td><td height=25px> <a href =\"q\">Decrease Min Pwm </a></td></tr><tr><td height=25px; bgcolor=#DAD7D7> <a href =\"r\">Increase MAX_PITCH Correction</a></td><td height=25px; bgcolor=#DAD7D7> <a href =\"s\">Decrease MAX_PITCH Correction </a></td></tr><tr><td height=25px> <a href =\"t\">Increase MAX_INTEGRAL ERROR</a></td><td height=25px> <a href =\"u\">Decrease MAX_INTEGRAL_ERROR </a></td></tr><tr><td height=25px; bgcolor=#DAD7D7> <a href =\"v\">Increase Setpoint</a></td><td height=25px; bgcolor=#DAD7D7> <a href =\"w\">Decrease Setpoint</a></td></tr></table></body></html>";

#define EXAMPLE_WIFI_SSID "SUA"
#define EXAMPLE_WIFI_PASS "12345678"

//////////////////////////////////////////////////////////////////////////////////////////////////////
//ALL VARIABLES BEYOND THIS 

float yaw_kP= 3;
float yaw_kI= 0;
float yaw_kD= 0.5;

float pitchKp=  5.85;       
float pitchKi=  0.095;          
float pitchKd=  2.8;

int MAX_PITCH_CORRECTION =90;
int MAX_INTEGRAL_ERROR= 90;
float MIN_OUT_FORWARD_ANGLE_CONSTRAIN = -2.5;

int MAX_PWM = 100; 
int MIN_PWM = 60;


float setpoint = -6;
float initial_acce_angle = -6;
float forward_angle = -4.5; 


////////////////////////////////////////////////////////////////////////////////////////////////////////// 

char pKp[10];
char pKd[10];
char pKi[10];
char yKp[10];
char yKd[10];
char yKi[10];
char MaxPwm[10];
char MinPwm[10];
char MaxPitchCorrection[10];
char MaxIntegralError[10];
char setPoint[15];
char enter[]="<br>";
char pitchArr[]="Pitch Parameters";
char PitchKpArr[]="Kp = ";
char PitchKdArr[]="Kd = ";
char PitchKiArr[]="Ki = ";
char yawArr[]= "Yaw Parameters";
char YawKpArr[]="Kp = ";
char YawKdArr[]="Kd = ";
char YawKiArr[]="Ki = ";
char MaxPwmArr[] = "Maximum Pwm = ";
char MinPwmArr[] = "Minimum Pwm = ";
char MaxPitchCorrectionArr[]= "Maximum Pitch Correction = ";
char MaxIntegralErrorArr[] = "Maximum Integral Error = ";
char setpointArr[] = "Setpoint = ";
char ForwardAngleArr[]= "Forward Angle";
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}
static void
http_server_netconn_serve(struct netconn *conn)
{
  struct netbuf *inbuf;
  char *buf;
  u16_t buflen;
  err_t err;

  /* Read the data from the port, blocking if nothing yet there.
   We assume the request (the part we care about) is in one netbuf */
  err = netconn_recv(conn, &inbuf);

  if (err == ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);

    /* Is this an HTTP GET command? (only check the first 5 chars, since
    there are other formats for GET, and we're keeping it very simple )*/
    if (buflen>=5 &&
        buf[0]=='G' &&
        buf[1]=='E' &&
        buf[2]=='T' &&
        buf[3]==' ' &&
        buf[4]=='/' ) {
          // printf("%c\n", buf[5]);
      /* Send the HTML header
             * subtract 1 from the size, since we dont send the \0 in the string
             * NETCONN_NOCOPY: our data is const static, so no need to copy it
       */
       

       if(buf[5]=='a'){
          pitchKp+=0.1;
       }
       if(buf[5]=='b'){
          pitchKp-=0.1;
       }
       if(buf[5]=='c'){
          pitchKd+=0.1;
       }
       if(buf[5]=='d'){
          pitchKd-=0.1;
       }
       if(buf[5]=='e'){
          pitchKi+=0.01;
       }
       if(buf[5]=='g'){
          pitchKi-=0.01;
       }
       if(buf[5]=='h'){
          yaw_kP+=0.1;
       }
       if(buf[5]=='i'){
          yaw_kP-=0.1;
       }
       if(buf[5]=='j'){
          yaw_kD+=0.1;
       }
       if(buf[5]=='k'){
          yaw_kD-=0.1;
       }
       if(buf[5]=='l'){
          yaw_kI+=0.01;
       }
       if(buf[5]=='m'){
          yaw_kI-=0.01;
       }
        if(buf[5]=='n'){
          MAX_PWM += 1;
       }
        if(buf[5]=='o'){
          MAX_PWM -= 1;
       }
        if(buf[5]=='p'){
          MIN_PWM+= 1;
       }
        if(buf[5]=='q'){
          MIN_PWM-= 1;
       }
        if(buf[5]=='r'){
          MAX_PITCH_CORRECTION+=1;
       }
        if(buf[5]=='s'){
          MAX_PITCH_CORRECTION-=1;
       }
        if(buf[5]=='t'){
          MAX_INTEGRAL_ERROR+=1;
       }
        if(buf[5]=='u'){
          MAX_INTEGRAL_ERROR-=1;
       }
        if(buf[5]=='v'){
          setpoint +=1;
       }
        if(buf[5]=='w'){
          setpoint-=1;
       }

        gcvt(pitchKp,5,pKp);
        gcvt(pitchKd,5,pKd);
        gcvt(pitchKi,5,pKi);
        gcvt(yaw_kP,5,yKp);
        gcvt(yaw_kD,5,yKd);
        gcvt(yaw_kI,5,yKi);
        gcvt(setpoint,5,setPoint);

        itoa(MAX_PWM, MaxPwm,10);
        itoa(MIN_PWM, MinPwm,10);
        itoa(MAX_PITCH_CORRECTION, MaxPitchCorrection,10);
        itoa(MAX_INTEGRAL_ERROR, MaxIntegralError,10);
        

        netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);


      /* Send our HTML page */
        netconn_write(conn, http_index_hml, sizeof(http_index_hml)-1, NETCONN_NOCOPY);
        netconn_write(conn, pitchArr, sizeof(pitchArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, PitchKpArr, sizeof(PitchKpArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, pKp, sizeof(pKp)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, PitchKdArr, sizeof(PitchKdArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, pKd, sizeof(pKd)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, PitchKiArr, sizeof(PitchKiArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, pKi, sizeof(pKi)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, yawArr, sizeof(yawArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, YawKpArr, sizeof(YawKpArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, yKp, sizeof(yKp)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, YawKdArr, sizeof(YawKdArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, yKd, sizeof(yKd)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, YawKiArr, sizeof(YawKiArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, yKi, sizeof(yKi)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, MaxPwmArr, sizeof(MaxPwmArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, MaxPwm, sizeof(MaxPwm)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, MinPwmArr, sizeof(MinPwmArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, MinPwm, sizeof(MinPwm)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, MaxPitchCorrectionArr, sizeof(MaxPitchCorrectionArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, MaxPitchCorrection, sizeof(MaxPitchCorrection)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, MaxIntegralErrorArr, sizeof(MaxIntegralErrorArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, MaxIntegralError ,sizeof(MaxIntegralError)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);
        netconn_write(conn, setpointArr, sizeof(setpointArr)-1 , NETCONN_NOCOPY);
        netconn_write(conn, setPoint, sizeof(setPoint)-1, NETCONN_NOCOPY);
        netconn_write(conn, enter, sizeof(enter)-1, NETCONN_NOCOPY);

      
      
    }

  }
  /* Close the connection (server closes in HTTP) */
  netconn_close(conn);

  /* Delete the buffer (netconn_recv gives us ownership,
   so we have to make sure to deallocate the buffer) */
  netbuf_delete(inbuf);
}


void wifiInit()
{
  nvs_flash_init();
  system_init();
  initialise_wifi();
}