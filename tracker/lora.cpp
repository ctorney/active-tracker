

#include "lora.h"


bool Lora::begin() {

  pinMode(LORA_IRQ_DUMB, OUTPUT);
//  SerialLoRa.begin(19200); 
//  while(!SerialLoRa);  
//  
  if (!activate()) 
    return false;
    
  delay(100);
  sendQuery("AT+VER?");
  sendQuery("AT+DEVEUI?");
  sendQuery("AT+APPEUI?");

  delay(100);

  
  sendCommand(String("AT+APPKEY=") + WBEEST_APP_KEY); // set appkey
  sendQuery("AT+APPKEY?");
  
  sendCommand("AT+MODE=1");
  
  
//  sendQuery("AT+DUTYCYCLE?");
//  Serial.println("delay windows");
//  sendQuery("AT+DELAY?");
//  sendQuery("AT+DELAY=5000,6000,5000,6000");
//  
//  Serial.println("new delay windows");
//  sendQuery("AT+DELAY?");
  

  sendCommand("AT+RTYNUM=3");
  sendQuery("AT+RTYNUM?");  
  join();
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



bool Lora::join(){

  bool joined = false;

  if (!activate()) 
    return joined;

    
  if (sendCommand("AT+JOIN")) // request join
  {  

    long lora_start_time = millis();
    long lora_timeout = 60*1000*5;  // break after 5 minutes
    String modem_ans;
    while (true)
    {
      Serial.println("waiting for join");
  
      modem_ans = SerialLoRa.readStringUntil('\r\n');
  
  
      if (modem_ans.startsWith("+OK")) {
        Serial.println("modem OK");
      }
      if (modem_ans.startsWith("+ERR")){
          Serial.print("modem error");
          Serial.println(modem_ans);
          
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
//  pinMode(LORA_IRQ_DUMB, OUTPUT);

  digitalWrite(LORA_IRQ_DUMB, LOW);
  lora_active=true;
  delay(500);
  return true;
}

void Lora::deactivate() {

  if (lora_active==false)
    return;

//  pinMode(LORA_IRQ_DUMB, OUTPUT);
  digitalWrite(LORA_IRQ_DUMB, HIGH);
//  
  delay(500);
  sendCommand("AT$DETACH"); // request UART to disconnect

//  long lora_start_time = millis();
//  long lora_timeout = 5000;  // break after 5 seconds
//  String modem_ans;
//  while (true)
//  {
//    modem_ans = SerialLoRa.readStringUntil('\r\n');
//    if (modem_ans.startsWith("+OK")) {
//      Serial.println("modem detach OK");
//      break;
//    }
//    if (millis() - lora_start_time > lora_timeout){
//      break;
//    }
//  }
//
//  SerialLoRa.end();
  lora_active=false;

  return;
}

void Lora::sendQuery(String atstring)
{

  Serial.print("Sending query: ");
  Serial.println(atstring);
  
  SerialLoRa.println(atstring);
  String answer;
  while (true)
  {
  
    answer = SerialLoRa.readStringUntil('\r\n');
    if (answer.startsWith("+OK")){
      break;
    }
    if (answer.startsWith("+ERR")){
      break;
    }
  }
  Serial.print("Response: ");
  Serial.println(answer);
 
}

bool Lora::sendCommand(String atstring)
{
  bool success = false;

  Serial.print("Sending command: ");
  Serial.println(atstring);

  SerialLoRa.println(atstring);
  
  String answer;
  while (true)
  {
  
    answer = SerialLoRa.readStringUntil('\r\n');
    if (answer.startsWith("+OK")){
      success=true;
      break;
    }
    if (answer.startsWith("+ERR")){
      success=false;
      break;
    }
  }
  Serial.print("Response: ");
  Serial.println(answer);
  return success;
}
