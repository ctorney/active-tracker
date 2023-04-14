#include "storage.h"

#include "CustoFlash.h"


void Storage::begin() {
  
    CustoFlash.beginWork();
    delay(500);
}
void Storage::sleep() {
  
//    CustoFlash.endWork();
    delay(500);
}

void Storage::write_next_message(location_reading location, activity_reading activity)
{



    uint8_t record_buffer[record_size];



    memcpy(record_buffer, &location, size_of_location); 
    memcpy(&record_buffer[size_of_location], &activity, size_of_activity); 

    RecordAddress_t recordAddr = CustoFlash.writeRecord(record_buffer, record_size);


}

bool Storage::anything_to_send()
{
 
  currentSectorIndex = CustoFlash.getEarliestBacklogSector();

  if (currentSectorIndex==NO_BACKLOG_SECTOR)
      return false;
  currentRecordIndex = CustoFlash.getEarliestBacklogIndex(currentSectorIndex);
  if (currentRecordIndex==NO_BACKLOG_RECORD)
      return false;

  RecordAddress_t recordAddr;

  recordAddr.sectorIndex = currentSectorIndex;
  recordAddr.recordIndex = currentRecordIndex;


  uint8_t recordSize = CustoFlash.readRecord(recordAddr, message_buffer);

  return true;

}

void Storage::send_successful()
{

  RecordAddress_t recordAddr;

  recordAddr.sectorIndex = currentSectorIndex;
  recordAddr.recordIndex = currentRecordIndex;

  CustoFlash.markRecordSent(recordAddr);

}
