#include <Servo.h>

#include "lora.h"


bool Lora::begin() {

  if (!activate()) 
    return false;
    
  delay(100);
  Serial.println("query modem version: ");
  at_query("AT+VER?");

  Serial.println("dev eui: ");
  at_query("AT+DEVEUI?");
  Serial.println("app eui: ");
  at_query("AT+APPEUI?");

  delay(100);
  SerialLoRa.flush();
  SerialLoRa.print("AT+APPKEY="); // set appkey
  SerialLoRa.println(WBEEST_APP_KEY); // set appkey

String answer;
  while (true)
  {
  
    answer = SerialLoRa.readStringUntil('\r\n');

  
  
    if (answer.startsWith("+OK")) break;
  }
  Serial.print("answer: ");
  Serial.println(answer);
  at_query("AT+APPKEY?");

  
  at_query("AT+DUTYCYCLE=0");
  at_query("AT+MODE=1");
  at_query("AT+DUTYCYCLE?");
  Serial.println("delay windows");
  at_query("AT+DELAY?");
  at_query("AT+DELAY=5000,6000,5000,6000");
  
  Serial.println("new delay windows");
  at_query("AT+DELAY?");
  
  Serial.println("RTYNUM");
  at_query("AT+RTYNUM?");
  SerialLoRa.print("AT+RTYNUM=3");
  

  deactivate();
  return true;
}



bool Lora::update(Storage* storage) {

  
 if (!storage->anything_to_send())
  return false;
 
 if (!join_success)
 {
    join_success = join();
    if (!join_success)
      return false;
 }

 LoraMessage message = storage->read_next_message();

 bool send_success = send_message(message);
 
 if (send_success)
  storage->send_successful();
 return send_success;
}

void Lora::at_query(String atstring)
{

  SerialLoRa.flush();
  SerialLoRa.println(atstring);
  String answer;
  while (true)
  {
  
    answer = SerialLoRa.readStringUntil('\r\n');
    if (answer.startsWith("+OK")) break;
  }
  Serial.print("answer: ");
  Serial.println(answer);

  
}


bool Lora::join(){

  bool joined = false;

  if (!activate()) 
    return joined;

  SerialLoRa.println("AT+JOIN"); // request UART to disconnect

  long lora_start_time = millis();
  long lora_timeout = 60*1000*5;  // break after 5 minutes
  String modem_ans;
  while (true)
  {
  
    modem_ans = SerialLoRa.readStringUntil('\r\n');


    if (modem_ans.startsWith("+OK")) {
      Serial.println("modem OK");
    }
    if (modem_ans.startsWith("+ERR")){
        Serial.println("modem error");
        // check error code 
        break;
    }
    if (modem_ans.startsWith("+EVENT=1,0")){
        Serial.println("no ack recv");
        // check error code 
        break;
    }
    if (modem_ans.startsWith("+EVENT=1,1")){
        Serial.println("ack recv");
        joined = true;
        break;
    }
    if (millis() - lora_start_time > lora_timeout){
      break;
    }
  }
  deactivate();
  delay(100);
  return joined;
}


bool Lora::send_message(LoraMessage message){

  bool message_sent = false;
  if (!activate()) 
    return message_sent;
    
  SerialLoRa.print("AT+PCTX 1,"); 
  SerialLoRa.print(message.getLength());
  SerialLoRa.print("\r"); 
  SerialLoRa.write(message.getBytes(),message.getLength()); 
//  SerialLoRa.println(""); 


  long lora_start_time = millis();
  long lora_timeout = 60*1000*5;  // break after 5 minutes
  String modem_ans;
  while (true)
  {
  
    modem_ans = SerialLoRa.readStringUntil('\r\n');


    if (modem_ans.startsWith("+OK")) {
      Serial.println("modem OK");
    }
    if (modem_ans.startsWith("+ERR")){
        Serial.println("modem error");
        // check error code 
        break;
    }
    if (modem_ans.startsWith("+NOACK")){
        Serial.println("no ack recv");
        // check error code 
        break;
    }
    if (modem_ans.startsWith("+ACK")){
        Serial.println("ack recv");
        message_sent = true;
        break;
    }
    if (millis() - lora_start_time > lora_timeout){
      break;
    }
  }
 
// Serial.println("Start sending..");   
//  for (int i=0;i<message.getLength();i++){
//    Serial.println(message.getBytes()[i],BIN);   
//  }
// Serial.println("End of message");   

  deactivate();
  delay(100);

  return message_sent;
  
}

bool Lora::activate() {

  if (lora_active==true)
    return true;
  SerialLoRa.begin(19200); 
  long lora_start_time = millis();
  long lora_timeout = 10000;
  while(!SerialLoRa){
    if (millis() - lora_start_time > lora_timeout)
      return false;
  }
  digitalWrite(LORA_IRQ_DUMB, LOW);
  lora_active=true;
  return true;
}

void Lora::deactivate() {

  if (lora_active==false)
    return;
  digitalWrite(LORA_IRQ_DUMB, HIGH);
  SerialLoRa.println("AT$DETACH"); // request UART to disconnect
  delay(100);
  SerialLoRa.end();
  lora_active=false;

  return;
}
