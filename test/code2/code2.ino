#include "kproject_dwm1000.h"

#include <SoftwareSerial.h>

SoftwareSerial soft(2, 3);

kproject_dwm1000 dwm;

void setup()
{
  Serial.begin(9600);
  soft.begin(9600);
  dwm.begin(soft); // dwm 모듈 시작

  dwm.set_anchor_mode(1); // 앵커모듈로 설정 앵커 번호 = 1
  //  dwm.set_tag_mode(11); // 태그모드로 설정 태그 번호 = 11

  dwm.set_anchor1_position(1, 0, 0);
  dwm.set_anchor2_position(2, 350, 0);
  dwm.set_anchor3_position(3, 350, 450);
}

void loop()
{
  int x, y;
  int dist1, dist2, dist3;
  int tag_no = 10;
  for ( tag_no = 10; tag_no < 20; tag_no++)
  {
    if ( dwm.get_position(tag_no, dist1, dist2, dist3, x, y) == true )
    {
      Serial.print("### POSITION ###");
      Serial.print(" TAG NO = ");
      Serial.print(tag_no);
      Serial.print(" , X = ");
      Serial.print(x);
      Serial.print(" , Y = ");
      Serial.print(y);
      Serial.print(" , DIST1 = ");
      Serial.print(dist1);
      Serial.print(" , DIST2 = ");
      Serial.print(dist2);
      Serial.print(" , DIST3 = ");
      Serial.print(dist3);
      Serial.println("");
    }
  }
  Serial.println("#####################");
}
