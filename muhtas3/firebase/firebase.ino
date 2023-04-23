#include "FirebaseESP8266.h" //getInt ile alıyoruz, setInt ile gönderiyoruz
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <FirebaseArduino.h>

//1. Firebase veritabanı adresini, Token bilgisini ve ağ adresi bilgilerinizi giriniz.
#define FIREBASE_HOST "iotsystems-76a65-default-rtdb.firebaseio.com/" // http:// veya https:// olmadan yazın
#define FIREBASE_AUTH "O0UCdzZxqKtk0OvPVv4mVgyoxxVpMXS6xlIRujFQ"
#define WIFI_SSID "HUAWEI Mate 20 lite"
#define WIFI_PASSWORD "eac6a6c8217f"


//2. veritabanim adında bir firebase veritabanı nesnesi oluşturuyoruz
FirebaseData veritabanim;

//SoftwareSerial s(D7,D8); //RX,TX STM-->PA9TX PA10RX
/*int ileri=0;
int geri=0;
int dur=0;
int right=0;
int left=0;
int accel=0;
int deccel=0;*/
int receiveddata[10];
  
void setup()
{

 // s.begin(115200);
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Ağ Bağlantısı Oluşturuluyor");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("IP adresine bağlanıldı: ");
  Serial.println(WiFi.localIP());
  Serial.println();


  //3. Firebase bağlantısı başlatılıyor

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  //4. Ağ bağlantısı kesilirse tekrar bağlanmasına izin veriyoruz
  Firebase.reconnectWiFi(true);
 pinMode(D2,OUTPUT);
 digitalWrite(D2,LOW);
}


void loop()
{
  if(Serial.available())
  {   
    byte gelenVeri[10];
    Serial.readBytes(gelenVeri,10);

    float sicaklik = gelenVeri[0]+(gelenVeri[1]*0.01);
    //receiveddata[10] = Serial.readBytes();
    Firebase.setInt("sicaklik", 43);
    for(int i=0; i<10;i++)
    {
      Serial.print(gelenVeri[i]);
    }
    
  }


  /*if(Firebase.getInt(veritabanim, "/forward")) //Alınacak veri tipine göre getInt, getBool, getFloat, getDouble, getString olarak kullanılabilir.
  {
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("forward = ");
    Serial.println(veritabanim.intData());
    ileri=Firebase.getInt(veritabanim, "/forward");
    if(ileri == 1)
    {
      Serial.write('1');
    }
  }
  
  if(Firebase.getInt(veritabanim, "/reverse")) 
  {
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("reverse = ");
    Serial.println(veritabanim.intData());
    geri=Firebase.getInt(veritabanim, "/reverse");
    if(geri == 1)
    {
       s.write('2');
    }
    
  }
  
  if(Firebase.getInt(veritabanim, "/stop")) 
  {
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("stop = ");
    Serial.println(veritabanim.intData());
    dur=Firebase.getInt(veritabanim, "/stop");
    if(dur == '1')
    {
    s.write('3');
    }
  }*/
}
