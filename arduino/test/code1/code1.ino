#include "kproject_dwm1000.h"

#include <SoftwareSerial.h>

SoftwareSerial soft(2, 3);

kproject_dwm1000 dwm;

void setup()
{
  Serial.begin(9600);
  soft.begin(9600);
  dwm.begin(soft); // dwm 모듈 시작

  //  dwm.set_anchor_mode(2); // 앵커모듈로 설정 앵커 번호 = 2
}

void loop()
{
  if ( Serial.available() )
  {
    int cmd = Serial.read();
    if ( cmd == '1' )
    {
      Serial.print("모듈 번호 : ");
      Serial.println(dwm.get_module_no()); // 모듈 번호를 구함
    }
    if ( cmd == '2' )
    {
      int tag_no = 10;
      Serial.print("현재 앵커와 태그 ");
      Serial.print(tag_no);
      Serial.print("와의 거리 = ");
      Serial.print(dwm.get_tag_distance(tag_no)); // 현재 앵커 모드과의 10번 태그 모듈간의 거리를 측정
      Serial.println("Cm");
    }
    if ( cmd == '3' )
    {
      int anchor_no = 2;
      int tag_no = 10;
      
      Serial.print(anchor_no);
      Serial.print("번 앵커와 태그 ");
      Serial.print(tag_no);
      Serial.print("와의 거리 = ");
      Serial.print(dwm.get_anchor_tag_distance(anchor_no, tag_no)); // 1번 앵커모듈과 10번 태그모 듈간의 거리를 측정
      Serial.println("Cm");
      
    }
    if ( cmd == '4' ) // 태그 모듈의 위치를 추적
    {
      int tag_no = 10;
      dwm.set_anchor1_position(1, 0, 0); // 앵커1 을 1번 앵커로 설정하고 좌표를 (350,0)으로 설정
      dwm.set_anchor2_position(2, 350, 0); // 앵커2 을 2번 앵커로 설정하고 좌표를 (0,0)으로 설정
      dwm.set_anchor3_position(3, 350, 450); // 앵커3 을 3번 앵커로 설정하고 좌표를 (350,450)으로 설정
      int x, y;
      int dist1, dist2, dist3;
      dwm.get_position(tag_no, dist1, dist2, dist3, x, y);
      Serial.print("태그 ");
      Serial.print(tag_no);
      Serial.print("의 좌표 = ");
      Serial.print("(");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.print(")");
      Serial.println("");
    }
  }
}
