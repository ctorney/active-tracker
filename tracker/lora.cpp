

#include "lora.h"


bool Lora::begin() {

  pinMode(LORA_IRQ_DUMB, OUTPUT);
  if (!activate()) 
    return false;
    
  sendQuery("AT+VER?");
  sendQuery("AT+DEVEUI?");
  sendQuery("AT+APPEUI?");

  sendCommand(String("AT+APPKEY=") + WBEEST_APP_KEY); // set appkey
  sendQuery("AT+APPKEY?");



  sendCommand("AT+MODE=1");
  sendCommand("AT+RTYNUM=3");
  sendCommand("AT+DELAY=5000,6000,5000,6000");
  sendCommand("AT+DUTYCYCLE=0");


  sendQuery("AT+RTYNUM?");  
  sendQuery("AT+DELAY?");  
  
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
  {
    storage->send_successful();
    session_success = true;
  }
  return ( send_success || session_success );

}



bool Lora::join(){

  bool joined = false;

  if (!activate()) 
    return joined;

   Serial.println("attempting to join..");

  int modem_status = sendCommand("AT+JOIN");
  if (modem_status==MODEM_OK) // join request sent
  {  

    long lora_start_time = millis();
    long lora_timeout = 60*1000*5;  // break after 5 minutes
    String modem_ans;
    while (true)
    {
  
      modem_ans = SerialLoRa.readStringUntil('\r\n');
  
      if (modem_ans.startsWith("+EVENT=1,0")){
          Serial.println("join failed");
          break;
      }
      if (modem_ans.startsWith("+EVENT=1,1")){
          Serial.println("join succeeded");
          joined = true;
          break;
      }
      if (millis() - lora_start_time > lora_timeout){
        break;
      }
    }
  }

  if (modem_status==ERR_ALREADY_JOINED) // already joined
    joined = true;

  
  
  deactivate();
  delay(100);
  return joined;
}


bool Lora::send_message(LoraMessage message){

  Serial.println("sending message");
  
  Serial.println(message.getLength());
  bool message_sent = false;
  if (!activate()) 
    return message_sent;

  // location messages are length 12 and go to port 3
  // activity messages are length 49 and go to port 5
  if (message.getLength()==12)
  {
    SerialLoRa.print("AT+PCTX 3,"); 
  }
  else
  {
    SerialLoRa.print("AT+PCTX 5,");
  }
  SerialLoRa.print(message.getLength());
  SerialLoRa.print("\r"); 
  SerialLoRa.write(message.getBytes(),message.getLength()); 

  int modem_status = 0;

  String answer;
  while (true)
  {
  
    answer = SerialLoRa.readStringUntil('\r\n');
    if (answer.startsWith("+OK")){
      modem_status=MODEM_OK;
      break;
    }
    if (answer.startsWith("+ERR")){
      modem_status = answer.substring(answer.indexOf("=")+1).toInt();
      break;
    }
  }

  if (modem_status==MODEM_OK)
  {
    long lora_start_time = millis();
    long lora_timeout = 60*1000*2;  // break after 2 minutes
    String modem_ans;
    while (true)
    {
    
      modem_ans = SerialLoRa.readStringUntil('\r\n');
      if (modem_ans.startsWith("+NOACK")){
          Serial.println("no ack recv");
          message_sent = false;

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
  }

 
  if (modem_status==ERR_NOT_JOINED) // already joined
    join_success = false;

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
  delay(500);
  return true;
}

void Lora::deactivate() {

  if (lora_active==false)
    return;
  
  digitalWrite(LORA_IRQ_DUMB, HIGH);
  
  delay(500);
  sendCommand("AT$DETACH"); // request UART to disconnect

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
  delay(500);
 
}

int Lora::sendCommand(String atstring)
{
  int modem_status = 0;

  Serial.print("Sending command: ");
  Serial.println(atstring);

  SerialLoRa.println(atstring);
  
  String answer;
  while (true)
  {
  
    answer = SerialLoRa.readStringUntil('\r\n');
    if (answer.startsWith("+OK")){
      modem_status=MODEM_OK;
      break;
    }
    if (answer.startsWith("+ERR")){
      // get the error code
      modem_status = answer.substring(answer.indexOf("=")+1).toInt();
      break;
    }
  }
  Serial.print("Response: ");
  Serial.println(answer);
  delay(500);
  return modem_status;
}
