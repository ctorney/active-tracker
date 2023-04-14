

#include "lora.h"


bool Lora::begin() {

  pinMode(LORA_IRQ_DUMB, OUTPUT);
  activate(); 
    
  sendQuery("AT+VER?");
  sendQuery("AT+DEVEUI?");
  sendQuery("AT+APPEUI?");

  sendCommand(String("AT+APPKEY=") + WBEEST_APP_KEY); // set appkey
  sendQuery("AT+APPKEY?");



  sendCommand("AT+MODE=1");
  sendCommand("AT+RTYNUM=8");
  sendCommand("AT+DELAY=5000,6000,5000,6000");
  sendCommand("AT+DUTYCYCLE=0");


  sendQuery("AT+RTYNUM?");  
  sendQuery("AT+DELAY?");  
  
  join(); 
  
  
  return true;
}



bool Lora::update(Storage* storage) {

  
 

  deactivate();
  storage->begin();
  Serial.println("checking if send needed");
  bool send_needed = storage->anything_to_send();
  storage->sleep();  
  
  if (!send_needed)
    return false;


  Serial.println("send needed");

  activate();

  
  if (!join_success)
  {
    join_success = join();
    if (!join_success)
      return false;
  }
    

  uint8_t* message_buffer = storage->message_buffer;


  bool activity_sent = false;

  if (!location_sent)
  {
    LoraMessage l_message;

    long location_time;
    float location_data[2];

    memcpy((uint8_t *)&location_time, message_buffer, sizeof(location_time)); 
    memcpy((uint8_t *)&location_data, &message_buffer[4], sizeof(location_data)); 


    l_message.addUnixtime(location_time);
    l_message.addLatLng(location_data[0],location_data[1]);


    location_sent = send_message(l_message);

  }
  if (location_sent)
  {
    Serial.println("sending location message");

    LoraMessage a_message;

    long start_time;
    byte activities[45];

    memcpy((uint8_t *)&start_time, &message_buffer[12], sizeof(start_time)); 
    memcpy((uint8_t *)&activities, &message_buffer[16], sizeof(activities)); 

    a_message.addUnixtime(start_time);
    for (int i=0;i<45;i++)
      a_message.addUint8(activities[i]);

    activity_sent = send_message(a_message);
  }
 
  if (activity_sent)
  {
    
    Serial.println("sending activity message");

    deactivate();
    location_sent=false;
    storage->begin();
    storage->send_successful();
    session_success = true;
    storage->sleep();
    activate();
  }

  return ( activity_sent || session_success );

}



bool Lora::join(){

  bool joined = false;
  
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

  
  return joined;
}


bool Lora::send_message(LoraMessage message){

  Serial.println("sending message");
  
  Serial.println(message.getLength());
  bool message_sent = false;

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

 
  if (modem_status==ERR_NOT_JOINED) // not joined
    join_success = false;


  return message_sent;
  
}

void Lora::activate() {

  if (lora_active==true)
    return;
  SerialLoRa.begin(19200); 
  long lora_start_time = millis();
  long lora_timeout = 10000;
  while(!SerialLoRa){
    if (millis() - lora_start_time > lora_timeout)
      return;
  }
  
  digitalWrite(LORA_IRQ_DUMB, LOW);
  lora_active=true;
  delay(500);
  return;
}

void Lora::deactivate() {

  if (lora_active==false)
    return;
  
  digitalWrite(LORA_IRQ_DUMB, HIGH);
  
  delay(500);
  sendCommand("AT$DETACH"); // request UART to disconnect

  lora_active=false;
  delay(500);

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
