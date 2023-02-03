# GPS-UWB-Camera-based-human-following
uwb 기반 팔로우 미
# 1. 의존성
**하드웨어**
1. DWM1000 + ESP32 납땜O [구매 링크](https://smartstore.naver.com/jy-soft/products/5552791624?NaPm=ct%3Dldnxpbg8%7Cci%3D1d49bf7a37f2cc0e76eef1c32811101537925050%7Ctr%3Dsls%7Csn%3D1174464%7Chk%3Dea5da422a12ada8b86d8ba1f837eaf0b042b9895)

2. 사물 인터넷 위모스 wemos D1 R32 ESP32 uno 우노 타입 보드 [구매 링크](https://smartstore.naver.com/makerspace/products/3901014085?NaPm=ct%3Dldnxrjp4%7Cci%3D8955809aa2480b3aeb569182facb3e42e1d8e731%7Ctr%3Dsls%7Csn%3D525290%7Chk%3D5f82223d3453d394b1cb04c868c4d52b369f5cad)
   + Qorvo DWS1000 [구매 링크](https://www.mouser.kr/ProductDetail/772-DWS1000)

**아두이노 라이브러리**
Esp32-SoftwareSerial-master
1. esp32 보드 설정
Preferences -> Additional Boards Manager URLs
https://dl.espressif.com/dl/package_esp32_index.json

2. esp32 보드 다운
tools -> board -> boards manager
esp32 by espressif systems

Rosserial_Arduino_Library-0.9.1
1. stetch -> include library  -> manage library
2. rosserial 검색 후 다운로드

arduino-dw1000-ng-master
1. https://github.com/F-Army/arduino-dw1000-ng 다운로드
1. stetch -> include library  -> add zip library
