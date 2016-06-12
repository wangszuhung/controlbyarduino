//=====UNO=====wangszuhung
//2016_01_21_13:00_新版CRC  入出檢查
#include<math.h>
#include<EEPROM.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

const boolean powerrelayPin = 5 ;
const boolean resistrelayPin = 2;
const boolean ledgreenPin = 3;
const boolean ledredPin = 4;
const boolean AddswitchWPin = A4;
const boolean AddswitchRPin = A5;
const int TempaiPin = A6;
const boolean buttonPin = 6;
const boolean AddePin = 11;//HIGH ADD
const boolean AdddPin = 10;
const boolean AddcPin = 9;
const boolean AddbPin = 8;
const boolean AddaPin = 7;//LOW ADD
unsigned int Address = 0;
boolean Adda, Addb, Addc, Addd, Adde;
boolean Resistmode, Powermode;
boolean errormode = 0;
int errorRead = 131;
int errorWrite = 134;
int TempvalueRead = 0 ;
//int Tempvalue = 0;
float Thermister = 0; 
int RS485in[8];
int RS485crc[8];
int RS485check[8];
int modbus_inout;
int addressLG=0,addressLR=1,addressSR=2,addressSP=3,addressMode =4,addressModeSta=5;
int temphigh = 0,templow = 0,timeairopen = 0, timeairclose = 0; 
unsigned long timeairopens,timeaircloses;
long timeairopensdiff=0,timeairclosesdiff=0;
boolean timeoc;
unsigned long currentTime = 0;

unsigned long startTime;
unsigned long duration;
boolean modestatus;
int modeslect;
unsigned long previousTimeMark;
int allbit = 0;
int Readcount;
boolean wr = 0;

static unsigned char auchCRCHi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40
};

static char auchCRCLo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
  0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
  0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
  0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
  0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
  0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
  0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
  0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
  0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
  0x40
};

const int temps[] PROGMEM = { 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 143, 144, 145, 146, 147, 148, 149, 150, 151, 151, 152, 153, 154, 155, 156, 157, 158, 159, 159, 160, 161, 162, 163, 164, 165, 166, 167, 167, 168, 169, 170, 171, 172, 173, 174, 175, 175, 176, 177, 178, 179, 180, 181, 182, 182, 183, 184, 185, 186, 187, 188, 189, 190, 190, 191, 192, 193, 194, 195, 196, 197, 197, 198, 199, 200, 201, 202, 203, 204, 205, 205, 206, 207, 208, 209, 210, 211, 212, 212, 213, 214, 215, 216, 217, 218, 219, 220, 220, 221, 222, 223, 224, 225, 226, 227, 228, 228, 229, 230, 231, 232, 233, 234, 235, 235, 236, 237, 238, 239, 240, 241, 242, 243, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 252, 253, 254, 255, 256, 257, 258, 259, 260, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 422, 423, 424, 425, 426, 427, 428, 429, 430, 432, 433, 434, 435, 436, 437, 438, 439, 441, 442, 443, 444, 445, 446, 448, 449, 450, 451, 452, 453, 455, 456, 457, 458, 459, 460, 462, 463, 464, 465, 466, 468, 469, 470, 471, 472, 474, 475, 476, 477, 479, 480, 481, 482, 484, 485, 486, 487, 489, 490, 491, 492, 494, 495, 496, 498, 499, 500, 501, 503, 504, 505, 507, 508, 509, 511, 512, 513, 515, 516, 517, 519, 520, 521, 523, 524, 525, 527, 528, 530, 531, 532, 534, 535, 537, 538, 539, 541, 542, 544, 545, 547, 548, 550, 551, 552, 554, 555, 557, 558, 560, 561, 563, 564, 566, 567, 569, 570, 572, 574, 575, 577, 578, 580, 581, 583, 585, 586, 588, 589, 591, 593, 594, 596, 598, 599, 601, 603, 604, 606, 608, 609, 611, 613, 614, 616, 618, 620, 621, 623, 625, 627, 628, 630, 632, 634, 636, 638, 639, 641, 643, 645, 647, 649, 651, 653, 654, 656, 658, 660, 662, 664, 666, 668, 670, 672, 674, 676, 678, 680, 683, 685, 687, 689, 691, 693, 695, 697, 700, 702, 704, 706, 708, 711, 713, 715, 718, 720, 722, 725, 727, 729, 732, 734, 737, 739, 741, 744, 746, 749, 752, 754, 757, 759, 762, 764, 767, 770, 773, 775, 778, 781, 784, 786, 789, 792, 795, 798, 801, 804, 807, 810, 813, 816, 819, 822, 825, 829, 832, 835, 838, 842, 845, 848, 852, 855, 859, 862, 866, 869, 873, 877, 881, 884, 888, 892, 896, 900, 904, 908, 912, 916, 920, 925, 929, 933, 938, 942, 947, 952, 956, 961, 966, 971, 976, 981, 986, 991, 997, 1002, 1007, 1013, 1019, 1024, 1030, 1036, 1042, 1049, 1055, 1061, 1068, 1075, 1082, 1088, 1096, 1103, 1110, 1118, 1126, 1134, 1142, 1150, 1159, 1168, 1177, 1186, 1196, 1206, 1216, 1226, 1237, 1248, 1260, 1272, 1284, 1297, 1310, 1324, 1338, 1353, 1369, 1385, 1402, 1420, 1439, 1459, 1480, 1502 };


void setup()
{
  pinMode(AddePin, INPUT);
  pinMode(AdddPin, INPUT);
  pinMode(AddcPin, INPUT);
  pinMode(AddbPin, INPUT);
  pinMode(AddaPin, INPUT);
  pinMode(ledredPin, OUTPUT);
  pinMode(ledgreenPin, OUTPUT);
  pinMode(AddswitchWPin, OUTPUT);
  pinMode(AddswitchRPin, OUTPUT);
  pinMode(powerrelayPin, OUTPUT);
  pinMode(resistrelayPin, OUTPUT);
  digitalWrite(powerrelayPin, LOW); 
  digitalWrite(resistrelayPin, LOW);
  digitalWrite(ledredPin, HIGH);
  digitalWrite(ledgreenPin, LOW);
  modestatus = 1; 
  modeslect = 2;
  temphigh = 27;
  templow = 23;
  timeairopen = 11;
  timeairclose = 3;
  timeoc = 0;
  previousTimeMark = millis();
  Addresstest();
  Temp();
  Serial.begin(9600);
}


void loop()
{
  unsigned long now = millis();
  wdt_enable(WDTO_8S);
  Serial.flush();
  if (Serial.available() > 0)
  {
    Modbus();
  }
  delayMicroseconds(5);
  
  if(digitalRead(buttonPin) == HIGH)
  {
    Serial.end();
    ButtonStatus();
    digitalWrite(AddswitchRPin, LOW);
    digitalWrite(AddswitchWPin, LOW);
    for (int i = 0; i < allbit ; i++)
    {
      RS485crc[i] = 0;
    }
    Serial.begin(9600);
  }
 
  if( modestatus ==1 && (now - previousTimeMark > 1000))
  {
    wdt_reset();
    updateTime(now - previousTimeMark);

    if(modeslect == 0)
    {
       modestatus = 0;
    }
    if(modeslect == 1)
    {
       ModeTemp();
    }
    if(modeslect == 2)
    {
      timeairopens = timeairopen * 60000;
      timeaircloses = timeairclose * 60000;
      ModeAirClOp();
    }
    if(modeslect == 3)
    {
        modestatus = 1; 
        modeslect = 2;
        
        temphigh = 27;
        templow = 23;
        timeairopen = 11;
        timeairclose = 3;
    }
    if(modeslect == 4)
    {
      wdt_enable(WDTO_2S);
    }
  }
}

void ButtonStatus()
{
    wdt_disable();
    startTime = millis();
    while (digitalRead(buttonPin) == HIGH);
    
    duration = millis() - startTime;
    {
      if(duration <= 1000)
      {
        for(int i=0;i<3;i++)
        {
          digitalWrite(ledgreenPin, LOW);
          digitalWrite(ledredPin, HIGH);
          delay(100);
          digitalWrite(ledgreenPin, HIGH);
          digitalWrite(ledredPin, LOW);
          delay(100);
        }
        delay(5);
        digitalWrite(powerrelayPin, LOW); 
        digitalWrite(resistrelayPin, LOW);
        digitalWrite(ledredPin, HIGH);
        digitalWrite(ledgreenPin, LOW);
        duration = 0;
        Addresstest();
        delay(5);
      }

      if((duration > 1000) && (duration <= 5000))
      {
        digitalWrite(ledredPin, HIGH); 
        digitalWrite(ledgreenPin, HIGH);
        for(int j=0; j<3; j++)
        {
          digitalWrite(ledgreenPin, LOW);
          delay(100);
          digitalWrite(ledgreenPin, HIGH);
          delay(100);
        }
        delay(10);
        duration = 0;
        wdt_enable(WDTO_1S);
        digitalWrite(powerrelayPin, LOW);
        digitalWrite(resistrelayPin, LOW);
        digitalWrite(ledredPin, HIGH);
        digitalWrite(ledgreenPin, LOW);
      }
    }
}
//wangszuhung
void Modbus()
{
  errormode = 0;
  for (int i = 0; i < 8; i++)
  {
    RS485crc[i] = 0;
    RS485crc[i] = Serial.read();
    delay(3);
  }
  RS485check[6] = RS485crc[6];
  RS485check[7] = RS485crc[7];
  modbus_inout = 0;
  allbit = 8;
  ModbusCRC();
  
  if((RS485check[6] == RS485crc[6]) && (RS485check[7] == RS485crc[7]))
  {
    for(int i =0;i<8;i++)
    {
      RS485in[i] = RS485crc[i];
    }
    
    if (RS485in[0] == Address)
    {
      wdt_disable();
      Serial.end();
      modbus_inout = 1;

      if (RS485in[1] == 3 || RS485in[1] == 6)
      {
        if (RS485in[1] == 3) 
        {
          ModbusRead();
        }
        if (RS485in[1] == 6) 
        {
          ModbusWrite();
        }
      }
      else
      {
        errormode = 1;
        functionerror();
      }
    }
  }
}
//wangszuhung
void ModbusRead()
{
  if (RS485in[4] == 0 && (RS485in[5] >= 1 && RS485in[5] <= 125)) 
  {
    if (RS485in[2] == 0 && (RS485in[3] >= 0 && RS485in[3] <= 7)) 
    {
      if (RS485in[4] == 0 && RS485in[5] == 1)
      {
        wr = 1;
        Readcount = RS485in[5];
        if (RS485in[2] == 0  && RS485in[3] == 0)
        {
          if (RS485in[4] == 0  && RS485in[5] == 1)
          {
            Rswitch();
          }
        }
        if (RS485in[2] == 0  && RS485in[3] == 1)
        {
          if (RS485in[4] == 0  && RS485in[5] == 1)
          {
            Pswitch();
          }
        }
        if (RS485in[2] == 0  && RS485in[3] == 2)
        {
          if (RS485in[4] == 0  && RS485in[5] == 1)
          {
            Temp();
          }
        }
        if (RS485in[2] == 0  && RS485in[3] == 3)
        {
          if (RS485in[4] == 0  && RS485in[5] == 1)
          {
            ModeStatus();
          }
        }
        if (RS485in[2] == 0  && RS485in[3] == 4)
        {
          if (RS485in[4] == 0  && RS485in[5] == 1)
          {
            TempHigh();
          }
        }
        if (RS485in[2] == 0  && RS485in[3] == 5)
        {
          if (RS485in[4] == 0  && RS485in[5] == 1)
          {
            TempLow();
          }
        }
        if (RS485in[2] == 0  && RS485in[3] == 6)
        {
          if (RS485in[4] == 0  && RS485in[5] == 1)
          {
            TimeAirOpen();
          }
        }
        if (RS485in[2] == 0  && RS485in[3] == 7)
        {
          if (RS485in[4] == 0  && RS485in[5] == 1)
          {
            TimeAirClose();
          }
        }
        delay(5);
        RS485crc[0] = RS485in[0];
        RS485crc[1] = RS485in[1];
        RS485crc[2] = 2;
        allbit = 7;
      }
      else
      {
        errormode = 1;
        RS485crc[0] = RS485in[0];
        RS485crc[1] = errorRead;
        RS485crc[2] = 4;
        allbit = 5;
      }
    }
    else
    {
      errormode = 1;
      RS485crc[0] = RS485in[0];
      RS485crc[1] = errorRead;
      RS485crc[2] = 2;
      allbit = 5;
    }
  }
  else
  {
    errormode = 1;
    RS485crc[0] = RS485in[0];
    RS485crc[1] = errorRead;
    RS485crc[2] = 3;
    allbit = 5;
  }
  ModbusCRC();
  delay(5);
  Modbusout();
}

void ModbusWrite() 
{
  if ((RS485in[4] >= 0 && RS485in[4] <= 255) || (RS485in[5] >= 0 && RS485in[5] <= 255))
  {
    if (RS485in[2] == 0 && (RS485in[3] >= 0 && RS485in[3] <= 7 && RS485in[3]!=2 ))
    {
      if (RS485in[4] == 0 && (RS485in[5] >= 0 && RS485in[5] <= 60)) 
      {
        wr = 0;
        if (RS485in[2] == 0  && RS485in[3] == 0)
        {
          modestatus = 0;
          Rswitch();
        }
        if (RS485in[2] == 0  && RS485in[3] == 1)
        {
          modestatus = 0;
          Pswitch();
        }
        
        if (RS485in[2] == 0  && RS485in[3] == 3) 
        {
          modestatus = 1;
          ModeStatus();
        }
        if (RS485in[2] == 0  && RS485in[3] == 4) 
        {
          modestatus = 1;
          TempHigh();
        }
        if (RS485in[2] == 0  && RS485in[3] == 5)
        {
          modestatus = 1;
          TempLow();
        }
        if (RS485in[2] == 0  && RS485in[3] == 6) 
        {
          modestatus = 1;
          TimeAirOpen();
        }
        if (RS485in[2] == 0  && RS485in[3] == 7)
        {
          modestatus = 1;
          TimeAirClose();
        }

        allbit = 8;
        for (int i = 0; i < 8; i++)
        {
          delay(1);
          RS485crc[i] = RS485in[i];
        }
      }
      else 
      {
        errormode = 1;
        RS485crc[0] = RS485in[0];
        RS485crc[1] = errorWrite;
        RS485crc[2] = 4;
        allbit = 5;
        ModbusCRC();
      }
    }
    else
    {
      errormode = 1;
      RS485crc[0] = RS485in[0];
      RS485crc[1] = errorWrite;
      RS485crc[2] = 2;
      allbit = 5;
      ModbusCRC(); 
    }
  }
  else
  {
    errormode = 1; 
    RS485crc[0] = RS485in[0];
    RS485crc[1] = errorWrite;
    RS485crc[2] = 3;
    allbit = 5;
    ModbusCRC();
  }
  delay(5);
  Modbusout();
}

void Modbusout()
{
  Serial.begin(9600);
  digitalWrite(AddswitchWPin, HIGH); 
  digitalWrite(AddswitchRPin, HIGH); 
  for (int i = 0; i < allbit ; i++)
  {
    Serial.write(RS485crc[i]);
    RS485crc[i] = 0;
  }
  delay(10);
  digitalWrite(AddswitchWPin, LOW); 
  digitalWrite(AddswitchRPin, LOW); 
}

void functionerror()
{
  RS485crc[0] = Address;
  RS485crc[1] = RS485in[1] + 128; //80 --16
  RS485crc[2] = 1;
  allbit = 5;
  ModbusCRC();
  Modbusout();
}

void ModeStatus()
{
  if( wr == 0)
  {
    if(RS485in[4] == 0 && RS485in[5] ==0) 
    {
      modestatus = 0;
      modeslect = 0;
    }
    if(RS485in[4] == 0 && RS485in[5] == 1) 
    {
      modeslect = 1;
    }
    if(RS485in[4] == 0 && RS485in[5] == 2) 
    {
      modeslect = 2;
    }
    if(RS485in[4] == 0 && RS485in[5] == 3)
    {
      modeslect = 3;
    }
    if(RS485in[4] == 0 && RS485in[5] == 4)
    {
      modeslect = 4;
    }
  }
  if( wr ==1 )
  {
    if(modeslect == 0)
    {
      RS485crc[3] = 0;
      RS485crc[4] = 0;
    }
    if(modeslect == 1)
    {
      RS485crc[3] = 0;
      RS485crc[4] = 1;
    }
    if(modeslect == 2) 
    {
      RS485crc[3] = 0;
      RS485crc[4] = 2;
    }
    if(modeslect == 3) 
    {
      RS485crc[3] = 0;
      RS485crc[4] = 3;
    }
    if(modeslect == 4) 
    {
      RS485crc[3] = 0;
      RS485crc[4] = 4;
    }
  }
}

void ModeTemp()
{
  Temp();
  if((Thermister/10) > temphigh) 
  {
    digitalWrite(powerrelayPin, LOW); 
    digitalWrite(resistrelayPin, LOW); 
    Resistmode = 0;
    digitalWrite(ledredPin, HIGH); 
    digitalWrite(ledgreenPin, LOW); 
  }
  if((Thermister/10) < templow) 
  {
    digitalWrite(powerrelayPin, LOW); 
    digitalWrite(resistrelayPin, HIGH); 
    Resistmode = 1;
    digitalWrite(ledgreenPin, HIGH); 
    digitalWrite(ledredPin, LOW);
  }
}

//boolean timeoc; //自動開關機用 0:OPEN  1:CLOSE
void ModeAirClOp() 
{
   if(timeairopensdiff < 0 && timeoc == 0){
      delayMicroseconds(100);
      currentTime = 0;
      timeoc = 1;
      digitalWrite(powerrelayPin, LOW); 
      digitalWrite(resistrelayPin, HIGH); 
      Resistmode = 1;
      digitalWrite(ledgreenPin, HIGH); 
      digitalWrite(ledredPin, LOW);
  }
  if(timeairclosesdiff < 0 && timeoc == 1){
      delayMicroseconds(100);
      currentTime = 0;
      timeoc = 0;
      digitalWrite(powerrelayPin, LOW); 
      digitalWrite(resistrelayPin, LOW); 
      Resistmode = 0;
      digitalWrite(ledgreenPin, LOW); 
      digitalWrite(ledredPin, HIGH);
  }
}

void updateTime(const unsigned long timePassed)
{
  previousTimeMark = millis();
  currentTime += timePassed;

  if(timeoc == 0 )
  {
    timeairclosesdiff =  timeaircloses;
    timeairopensdiff =  timeairopens - currentTime;
  }
  if(timeoc == 1 )
  {
    timeairopensdiff =  timeairopens;
    timeairclosesdiff =  timeaircloses - currentTime;
  }
}

void TempHigh()
{
  if( wr == 0)
  {
    temphigh = RS485in[5];
  }
  if( wr == 1)
  {
    if( temphigh < 256 )
    {
      RS485crc[3] = 0;
      RS485crc[4] = temphigh ;
    }
    if( temphigh > 256 )
    {
      RS485crc[3] = (temphigh / 256);
      RS485crc[4] = (temphigh % 256);
    }
  }
}

void TempLow()
{
  if( wr == 0)
  {
   templow = RS485in[5];
  }
  if( wr == 1)
  {
    if( templow < 256 )
    {
      RS485crc[3] = 0;
      RS485crc[4] = templow ;
    }
    if( temphigh > 256 )
    {
      RS485crc[3] = (templow / 256);
      RS485crc[4] = (templow % 256);
    }
  }
}

void TimeAirOpen()
{
  if( wr == 0)
  {
   timeairopen = RS485in[5];
  }
  if( wr == 1)
  {
    if( timeairopen < 256 )
    {
      RS485crc[3] = 0;
      RS485crc[4] = timeairopen ;
    }
    if( timeairopen > 256 )
    {
      RS485crc[3] = (timeairopen / 256);
      RS485crc[4] = (timeairopen % 256);
    }
  }
}

void TimeAirClose()
{
  if( wr == 0)
  {
   timeairclose = RS485in[5];
  }
  if( wr == 1)
  {
    if( timeairclose < 256 )
    {
      RS485crc[3] = 0;
      RS485crc[4] = timeairclose ;
    }
    if( timeairclose > 256 )
    {
      RS485crc[3] = (timeairclose / 256);
      RS485crc[4] = (timeairclose % 256);
    }
  }
}

void Rswitch()
{
  if ((RS485in[4] == 0  && RS485in[5] == 0) && wr == 0) 
  {
    delay(5);//
    digitalWrite(powerrelayPin, LOW); 
    digitalWrite(resistrelayPin, LOW);
    Resistmode = 0;
    digitalWrite(ledredPin, HIGH);  
    digitalWrite(ledgreenPin, LOW); 
  }
  if ((RS485in[4] == 0  && RS485in[5] == 1) && wr == 0)
  {
    delay(5);
    digitalWrite(powerrelayPin, LOW); 
    digitalWrite(resistrelayPin, HIGH); 
    Resistmode = 1;
    digitalWrite(ledgreenPin, HIGH); 
    digitalWrite(ledredPin, LOW); 
  }
  if (wr == 1)
  {
    if (digitalRead(resistrelayPin) == LOW)
    {
      RS485crc[3] = 0;
      RS485crc[4] = 0;
    }
    if (digitalRead(resistrelayPin) == HIGH) 
    {
      RS485crc[3] = 0;
      RS485crc[4] = 1;
    }
  }
}

void Pswitch() 
{
  if ((RS485in[4] == 0  && RS485in[5] == 0) && wr == 0 ) 
  {
    delay(5);
    digitalWrite(powerrelayPin, LOW); 
    digitalWrite(resistrelayPin, LOW); 
    digitalWrite(ledredPin, HIGH);  
    digitalWrite(ledgreenPin, LOW); 
    Powermode = 1;
  }
  if ((RS485in[4] == 0  && RS485in[5] == 1) && wr == 0) 
  {
    delay(5);
    digitalWrite(powerrelayPin, HIGH); 
    Powermode = 0;
    digitalWrite(ledgreenPin, LOW); 
    digitalWrite(ledredPin, LOW); 
  }
  if ( wr == 1 )
  {
    if (digitalRead(powerrelayPin) == HIGH) 
    {
      RS485crc[3] = 0;
      RS485crc[4] = 1;
    }
    if (digitalRead(powerrelayPin) == LOW)
    {
      RS485crc[3] = 0;
      RS485crc[4] = 0;
    }
  }
}

void ModbusCRC()
{
  unsigned char uchCRCHi = 0xFF ; 
  unsigned char uchCRCLo = 0xFF ; 
  unsigned int uIndex ; 
  int i = 0;
  int allbitt = allbit - 2;
  while (allbitt--) 
  {
    uIndex = uchCRCHi ^ RS485crc[i++] ;
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex] ;

  }
  if (((uchCRCHi << 8 ) | uchCRCLo) == 0x00)
  {
    return;
  }
  else
  {
    if(modbus_inout == 0)
    {
        RS485crc[7] = uchCRCLo;
        RS485crc[6] = uchCRCHi;
    }
    else if(modbus_inout == 1) 
    {
      if ( wr == 0) 
      {
        RS485crc[7] = uchCRCLo;
        RS485crc[6] = uchCRCHi;
      }
      if ( wr == 1) 
      {
        RS485crc[6] = uchCRCLo;
        RS485crc[5] = uchCRCHi;
      }
      if ( errormode == 1)
      {
        RS485crc[4] = uchCRCLo;
        RS485crc[3] = uchCRCHi;
      }
    }
  }
}

void Addresstest()  
{
  Address = 0;
  Adda = digitalRead(AddaPin);
  Addb = digitalRead(AddbPin);
  Addc = digitalRead(AddcPin);
  Addd = digitalRead(AdddPin);
  Adde = digitalRead(AddePin);
  delay(3);

  if (Adda == LOW) // 1
  {
    Address = Address + 1 ;
  }
  if (Addb == LOW) //2
  {
    Address = Address + 2 ;
  }
  if (Addc == LOW) //4
  {
    Address  = Address + 4;
  }
  if (Addd == LOW) //8
  {
    Address  = Address + 8;
  }
  if (Adde == 0) //16
  {
    Address = Address + 16;
  }
}

void Temp() 
{
  /*
  TempvalueRead = analogRead(TempaiPin);
  Thermister = log(((10240000 / TempvalueRead) - 10000));
  Thermister = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Thermister * Thermister )) * Thermister );
  Thermister = Thermister - 273.15; 
  Thermister = ((Thermister * 9.0) / 5.0 + 32.0 - 50)*10; */
  TempvalueRead = analogRead(TempaiPin)-239;
  Thermister = pgm_read_word(&temps[TempvalueRead]);
  if ( wr == 1 )
  {
    if ((int)Thermister < (int)256)
    {
      RS485crc[3] = 0 ;
      RS485crc[4] = ((int)Thermister);
    }
    if ((int)Thermister >= (int)256)
    {
      RS485crc[3] = (((int)Thermister) / (int)256);
      RS485crc[4] = (((int)Thermister) % (int)256);
    }
  }
}
//wangszuhung
