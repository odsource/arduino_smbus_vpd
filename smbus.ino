#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5
#include SoftI2CMaster.h
#include SPI.h
#include Adafruit_GFX.h
 standard I2C address
byte deviceAddress;
int index = 0;

 Standard and common non-standard Smart Battery commands
#define bufferLen 256

struct VPD_HEADER {
  byte IPMIVER;
  byte IUAOFF;
  byte CIAOFF;
  byte BIAOFF;
  byte PIAOFF;
  byte MRIOFF;
  byte reserved;
  byte CHCHK;
};

struct PRODUCT_INFO_AREA {
  byte IPMIVER;
  byte PALEN;
  byte LCODE;
  byte MNTL;
  byte MNAME[8];
  byte PNTL;
  byte PNAME[24];
  byte PPMNNTL;
  byte PPMN[40];
  byte PVTL;
  byte PVER[2];
  byte PSNTL;
  byte PSN[20];
  byte Asset_ATTL;
  byte AT;
  byte FRU_ATTL;
  byte FFI;
  byte CPIA;
  byte CPIA_ADD;
  byte EOR;
  byte align;
  byte PICHK;
};

void setup()
{
  Serial.begin(9600);
  delay(1000);
  while (!Serial) {    
    ;  wait for Serial port to connect.
  }

  Serial.println(Serial Initialized);

  bool init = i2c_init();
  Serial.println(init);
  if (!init)
  {
    Serial.println(I2C initialization failed);
    while(true)
    {
      continue;
    }
  }
  Serial.println(I2C Inialized);
  scan(); 
  delay(5000);
}

int fetchWord(byte func)
{
  i2c_start(deviceAddress1  I2C_WRITE);
  i2c_write(func);
  i2c_rep_start(deviceAddress1  I2C_READ);
  byte b1 = i2c_read(false);
  byte b2 = i2c_read(true);
  i2c_stop();
  return (int)b1((( int)b2)8);
}

uint8_t i2c_smbus_random_read_block ( uint8_t address, uint8_t blockBuffer, uint8_t blockBufferLen ) 
{
  uint8_t x, num_bytes = 0;
  if (!i2c_start((deviceAddress1) + I2C_WRITE))
  {
    Serial.println(Start was not acknowledged);
    i2c_stop();
    return 0;
  }
  if (!i2c_write(address))
  {
    Serial.println(Write was not acknowledged);
    i2c_stop();
    return 0;
  }
  if (!i2c_rep_start((deviceAddress1) + I2C_READ))
  {
    Serial.println(Repetition of start was not acknowledged);
    i2c_stop();
    return 0;
  }
  i2c_read(false);  num of bytes; 1 byte will be index 0
  for (x=0; xblockBufferLen; x++) {  -1 because x=num_bytes-1 if xy; last byte needs to be nack'd, xy-1
    blockBuffer[x] = i2c_read(false);
  }
  blockBuffer[x++] = i2c_read(true);  this will nack the last byte and store it in x's num_bytes-1 address.
  blockBuffer[x] = 0;  and null it at last_byte+1
  i2c_stop();
  return num_bytes;
}

uint8_t i2c_smbus_simple_read_block (uint8_t blockBuffer, uint8_t blockBufferLen ) 
{
  uint16_t x, num_bytes;
  if (!i2c_start((deviceAddress1) + I2C_READ))
  {
    Serial.println(Start was not acknowledged);
    i2c_stop();
    return 0;
  }
  for (x=0; x=254; x++)
  {
    blockBuffer[x] = i2c_read(false);
  }
  blockBuffer[x++] = i2c_read(true);  this will nack the last byte and store it in x's num_bytes-1 address.
  blockBuffer[x] = 0;  and null it at last_byte+1
  i2c_stop();
  return num_bytes;
}

void scan()
{
  byte i = 0;
  for ( i= 0; i  128; i++  )
  {
    bool ack = i2c_start(i1  I2C_WRITE); 
    if ( ack ) {
      Serial.print(Address 0x);
      Serial.print(i,HEX);
      if (((i  1) & 0xF0) != 0x60)
      { 
        deviceAddress = i;
        Serial.print( will be used);
      }

      Serial.println( OK);
      Serial.flush();
    }
    else {
   
    }
    i2c_stop();
  }
}

VPD_HEADER printVpdHeader(uint8_t i2cBuffer[])
{
  byte pHead;
  struct VPD_HEADER vpd_header;
  pHead = i2cBuffer;
  vpd_header = (VPD_HEADER) pHead; = (struct VPD_HEADER) pHead;
  Serial.println(VPD Header);
  Serial.print(    IPMI Format Version Number (IPMIVER) 0x);
  Serial.println(vpd_header-IPMIVER, HEX);
  Serial.print(    Internal Use Area Starting Offset (IUAOFF) 0x);
  Serial.println(vpd_header-IUAOFF, HEX);
  if (vpd_header-IUAOFF == 0)
  {
    Serial.println(    - not available);
  }
  Serial.print(    Chassis Info Area Starting Offset (CIAOFF) 0x);
  Serial.println(vpd_header-CIAOFF, HEX);
  if (vpd_header-CIAOFF == 0)
  {
    Serial.println(    - not available);
  }
  Serial.print(    Board Info Area Starting Offset (BIAOFF) 0x);
  Serial.println(vpd_header-BIAOFF, HEX);
  if (vpd_header-BIAOFF == 0)
  {
    Serial.println(    - not available);
  }
  Serial.print(    Product Info Area Starting Offset (PIAOFF) 0x);
  Serial.println(vpd_header-PIAOFF, HEX);
  if (vpd_header-PIAOFF == 0)
  {
    Serial.println(    - not available);
  }
  Serial.print(    MultiRecord Info Area Starting Offset (MRIOFF) 0x);
  Serial.println(vpd_header-MRIOFF, HEX);
  if (vpd_header-MRIOFF == 0)
  {
    Serial.println(    - not available);
  }
  Serial.print(    Reserved 0x);
  Serial.println(vpd_header-reserved, HEX);
  Serial.print(    Common Header Checksum (CHCHK) 0x);
  Serial.println(vpd_header-CHCHK, HEX);
  pHead+=sizeof(VPD_HEADER);
  return vpd_header;
}

void printIUAOFF(uint8_t offset, uint8_t i2cBuffer[])
{
  byte pHead;
  pHead = i2cBuffer;
  pHead+=offset;
  Serial.println(Internal Use Area);
  Serial.print(    );
  Serial.println();
}

void printCIAOFF(uint8_t offset, uint8_t i2cBuffer[])
{
  byte pHead;
  pHead = i2cBuffer;
  pHead+=offset;
  Serial.println(Chassis Info Area);
  Serial.print(    );
  Serial.println();
}

void printBIAOFF(uint8_t offset, uint8_t i2cBuffer[])
{
  byte pHead;
  pHead = i2cBuffer;
  pHead+=offset;
  Serial.println(Board Info Area);
  Serial.print(    );
  Serial.println();
}

void printPIAOFF(uint8_t offset, uint8_t i2cBuffer[])
{  
  byte pHead;
  struct PRODUCT_INFO_AREA pProduct_info_area;
  byte len = 0;
  pHead = i2cBuffer;
  Serial.println(Product Info Area);
  Serial.print(    Offset );
  Serial.print(offset);
  Serial.println( bytes);
  pHead+=offset;
  pProduct_info_area.IPMIVER = pHead++;
  Serial.print(    IPMI Format Version Number (IPMIVER) 0x);
  Serial.println(pProduct_info_area.IPMIVER, HEX);
  pProduct_info_area.PALEN = pHead++;
  Serial.print(    Product Info Area Length (PALEN) 0x);
  Serial.println(pProduct_info_area.PALEN, HEX);
  pProduct_info_area.LCODE = pHead++;
  Serial.print(    Language Code (LCODE) 0x);
  Serial.println(pProduct_info_area.LCODE, HEX);
  pProduct_info_area.MNTL = pHead++;
  Serial.print(    Manufacturer Name TypeLength (MNTL) 0x);
  Serial.println(pProduct_info_area.MNTL, HEX);
  Serial.print(    Manufacturer Name (MNAME) );
  len = (pProduct_info_area.MNTL & 0x3F);
  for (byte i = 0; i  len; i++)
  {
    pProduct_info_area.MNAME[i] = pHead++;
  }
  Serial.write(pProduct_info_area.MNAME, len);
  Serial.println();
  
  pProduct_info_area.PNTL = pHead++;
  Serial.print(    Product Name TypeLength (PNTL) 0x);
  Serial.println(pProduct_info_area.PNTL, HEX);
  Serial.print(    Product Name (PNAME) );
  len = (pProduct_info_area.PNTL & 0x3F);
  for (byte i = 0; i  len; i++)
  {
    pProduct_info_area.PNAME[i] = pHead++;
  }
  Serial.write(pProduct_info_area.PNAME, len);
  Serial.println();
  
  pProduct_info_area.PPMNNTL = pHead++;
  Serial.print(    Product PartModel Number TypeLength (PPMNNTL) 0x);
  Serial.println(pProduct_info_area.PPMNNTL, HEX);
  Serial.print(    Product PartModel Number (PPMN) );
  len = (pProduct_info_area.PPMNNTL & 0x3F);
  for (byte i = 0; i  len; i++)
  {
    pProduct_info_area.PPMN[i] = pHead++;
  }
  Serial.write(pProduct_info_area.PPMN, len);
  Serial.println();

  pProduct_info_area.PVTL = pHead++;
  Serial.print(    Product Version TypeLength (PVTL) 0x);
  Serial.println(pProduct_info_area.PVTL, HEX);
  Serial.print(    Product Version (PVER) );
  len = (pProduct_info_area.PVTL & 0x3F);
  for (byte i = 0; i  len; i++)
  {
    pProduct_info_area.PVER[i] = pHead++;
  }
  Serial.write(pProduct_info_area.PVER, len);
  Serial.println();
  
  pProduct_info_area.PSNTL = pHead++;
  Serial.print(    Product Serial Number TypeLength (PSNTL) 0x);
  Serial.println(pProduct_info_area.PSNTL, HEX);
  Serial.print(    Product Serial Number (PSN) );
  len = (pProduct_info_area.PSNTL & 0x3F);
  for (byte i = 0; i  len; i++)
  {
    pProduct_info_area.PSN[i] = pHead++;
  }
  Serial.write(pProduct_info_area.PSN, len);
  Serial.println();

  pProduct_info_area.Asset_ATTL = pHead++;
  Serial.print(    Asset Tag TypeLength (ATTL) 0x);
  Serial.println(pProduct_info_area.Asset_ATTL, HEX);
  Serial.print(    Asset Tag (AT) );
  len = (pProduct_info_area.Asset_ATTL & 0x3F);
  if (len == 0)
  {
    Serial.println(Asset Tag not present);
  }
  else
  {
    for (byte i = 0; i  len; i++)
    {
      pProduct_info_area.AT[i] = pHead++;
    }
    Serial.write(pProduct_info_area.AT, len);
    Serial.println();
  }

  pProduct_info_area.FRU_ATTL = pHead++;
  Serial.print(    FRU File ID TypeLength (ATTL) 0x);
  Serial.println(pProduct_info_area.FRU_ATTL, HEX);
  Serial.print(    FRU File ID (FFI) );
  len = (pProduct_info_area.FRU_ATTL & 0x3F);
  if (len == 0)
  {
    Serial.println(FRU File ID not present);
  }
  else
  {
    for (byte i = 0; i  len; i++)
    {
      pProduct_info_area.FFI[i] = pHead++;
    }
    Serial.write(pProduct_info_area.FFI, len);
    Serial.println();
  }

  pProduct_info_area.CPIA = pHead++;
  if (pProduct_info_area.CPIA == 0xC1)
  {
    pProduct_info_area.EOR = pProduct_info_area.CPIA;
    Serial.print(    End of Record (EOR) 0x);
    Serial.println(pProduct_info_area.EOR, HEX);
    if (pProduct_info_area.EOR == 0xC1)
    {
      Serial.print(    End of record reached);
    }
    Serial.println();
  }
  else
  {
    Serial.print(    Custom Product Info Area (CPIA) 0x);
    Serial.println(pProduct_info_area.CPIA, HEX);
    len = (pProduct_info_area.CPIA & 0x3F);
    if (len == 0)
    {
      Serial.println(    No Custom Product Info Area available);
    }
    else
    {
      for (byte i = 0; i  len; i++)
      {
        pProduct_info_area.CPIA_ADD[i] = pHead++;
      }
      Serial.write(pProduct_info_area.CPIA_ADD, len);
    }
  
    pProduct_info_area.EOR = pHead++;
    Serial.print(    End of Record (EOR) 0x);
    Serial.println(pProduct_info_area.EOR, HEX);
    if (pProduct_info_area.EOR == 0xC1)
    {
      Serial.print(    End of record reached);
    }
    Serial.println();
  }
  while(pHead++ == 0)
  {
    continue;
  }

  pProduct_info_area.PICHK = pHead++;
  Serial.print(    Product Info Area (PICHK) 0x);
  Serial.println(pProduct_info_area.PICHK, HEX);
  int checksum = 0;
  pHead = i2cBuffer;
  pHead+=offset;
  for (int i = 0; i  pProduct_info_area.PALEN - 1; i++)
  {
    checksum += pHead++;
  }
  checksum = checksum % 256;
  checksum = checksum - pProduct_info_area.PICHK;
  if (checksum != 0)
  {
    Serial.print(    Checksum wrong!);
  }
  else
  {
    Serial.print(    Chekcsum valid!);
  }
  Serial.println();

}

void printMRIOFF(uint8_t offset, uint8_t i2cBuffer[])
{
  byte pHead;
  pHead = i2cBuffer;
  pHead+=offset;
  Serial.println(MultiRecord Info Area);
  Serial.print(    );
  Serial.println();
}

void printCHCHK(uint8_t i2cBuffer[])
{
  byte pHead;
  Serial.println(Common Header Checksum);
  Serial.print(    );
  Serial.println();
}

void printVpd(uint8_t i2cBuffer[])
{
  VPD_HEADER vpd_header = printVpdHeader(i2cBuffer);
  delay(500);
  if (vpd_header.IUAOFF != 0)
  {
    printIUAOFF(vpd_header.IUAOFF  8, i2cBuffer);
    delay(500);
  }
  if (vpd_header.CIAOFF != 0)
  {
    printCIAOFF(vpd_header.CIAOFF  8, i2cBuffer);
    delay(500);
  }
  if (vpd_header.BIAOFF != 0)
  {
    printBIAOFF(vpd_header.BIAOFF  8, i2cBuffer);
    delay(500);
  }
  if (vpd_header.PIAOFF != 0)
  {
    printPIAOFF(vpd_header.PIAOFF  8, i2cBuffer);
    delay(500);
  }
  if (vpd_header.MRIOFF != 0)
  {
    printMRIOFF(vpd_header.MRIOFF  8, i2cBuffer);
    delay(500);
  }
  printCHCHK(i2cBuffer);
  Serial.println();
}

void loop()
{
  uint8_t i2cBuffer[bufferLen];
  uint8_t length_read = 0;
  Serial.print(Start for device address 0x);
  Serial.println(deviceAddress, HEX);
  i2c_smbus_simple_read_block(i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, 255);
  Serial.println();  
  printVpd(i2cBuffer);
  delay(1000);
  Serial.println(Hex dump );
  Serial.print(       0);
  for (uint8_t i = 1; i  8; i++)
  {
    Serial.print(     );
    Serial.print(i);
  }
  Serial.println();
  for (uint8_t j = 0; j  32; j++)
  {
    if (j  16) { Serial.print(0);}
    Serial.print(j, HEX);
    Serial.print(    );
    for (int i = 0; i  8; i++)
    {
      Serial.print(0x);
      if (i2cBuffer[8j + i]  16)
      {
        Serial.print(0);
      }
      Serial.print(i2cBuffer[8j + i], HEX);
      Serial.print(  );
    }
    Serial.println();
  }
  index++;
  while(true)
  {
    continue;
  }
  delay(2000);
}