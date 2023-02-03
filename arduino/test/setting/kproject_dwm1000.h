#define RETRY_NO 3

class kproject_dwm1000
{
  public:
    int time_out;
    void begin(Stream &stream);
    void set_anchor_mode(int no);
    void set_tag_mode(int no);
    int get_module_no();
    int _get_tag_distance(int tag_no);
    int get_tag_distance(int tag_no);
    int _get_anchor_tag_distance(int anchor_no, int tag_no);
    int get_anchor_tag_distance(int anchor_no, int tag_no);
    void set_anchor1_position(int anchor_id, int x, int y);
    void set_anchor2_position(int anchor_id, int x, int y);
    void set_anchor3_position(int anchor_id, int x, int y);
    void _get_position(int tag_id, int &dist1, int &dist2, int &dist3, int &x, int &y);
    bool get_position(int tag_id, int &dist1, int &dist2, int &dist3, int &x, int &y);

  public:
    Stream *_stream;
};

void kproject_dwm1000::begin(Stream &stream)
{
  time_out = 100;
  _stream = &stream;
}

void kproject_dwm1000::set_anchor_mode(int no)
{
  _stream->write(20);
  _stream->write(1);
  _stream->write(no);
  _stream->write(21);
  delay(1000); // 모듈 리셋되므로 대기
}

void kproject_dwm1000::set_tag_mode(int no)
{
  _stream->write(20);
  _stream->write(2);
  _stream->write(no);
  _stream->write(21);
  delay(1000); // 모듈 리셋되므로 대기
}

int kproject_dwm1000::get_module_no()
{
  while (_stream->available()) _stream->read();
  _stream->write(20);
  _stream->write(3);
  _stream->write(21);
  unsigned long start_millis = millis();
  while (_stream->available() < 5)
  {
    if ( ( millis() - start_millis ) > time_out ) return -1;
  }
  if ( _stream->read() != 20 ) return -1;
  if ( _stream->read() != 3 ) return -1;

  int mode = _stream->read();
  if ( ( mode != 0 ) && ( mode != 1 ) ) return -1;
  int value = _stream->read();
  if ( _stream->read() != 21 ) return -1;
  return value;
}

int kproject_dwm1000::_get_tag_distance(int tag_no)
{
  while (_stream->available()) _stream->read();
  _stream->write(20);
  _stream->write(4);
  _stream->write(tag_no);
  _stream->write(21);
  unsigned long start_millis = millis();
  while (_stream->available() < 5)
  {
    if ( ( millis() - start_millis ) > time_out ) return -1;
  }
  if ( _stream->read() != 20 ) return -1;
  if ( _stream->read() != 4 ) return -1;

  int distance = _stream->read();
  distance = distance | (_stream->read() << 8);
  if ( _stream->read() != 21 ) return -1;
  return distance;
}

int kproject_dwm1000::get_tag_distance(int tag_no)
{
  for ( int i = 0; i < RETRY_NO; i++)
  {
    int value =  _get_tag_distance(tag_no);
    if ( value != -10000 ) return value;
  }
  return -10000;
}

int kproject_dwm1000::_get_anchor_tag_distance(int anchor_no, int tag_no)
{
  while (_stream->available()) _stream->read();
  _stream->write(20);
  _stream->write(11);
  _stream->write(anchor_no);
  _stream->write(tag_no);
  _stream->write(21);

  unsigned long start_millis = millis();
  while (_stream->available() < 5)
  {
    if ( ( millis() - start_millis ) > time_out ) return -1;
  }
  if ( _stream->read() != 20 ) return -1;
  if ( _stream->read() != 4 ) return -1;

  int distance = _stream->read();
  distance = distance | (_stream->read() << 8);
  if ( _stream->read() != 21 ) return -1;
  return distance;
}

int kproject_dwm1000::get_anchor_tag_distance(int anchor_no, int tag_no)
{
  for ( int i = 0; i < RETRY_NO; i++)
  {
    int value =  _get_anchor_tag_distance(anchor_no, tag_no);
    if ( value != -10000 ) return value;
  }
  return -10000;
}

void kproject_dwm1000::set_anchor1_position(int anchor_id, int x, int y)
{
  _stream->write(20);
  _stream->write(8);
  _stream->write(anchor_id);
  _stream->write(x);
  _stream->write(x >> 8);
  _stream->write(y);
  _stream->write(y >> 8);
  _stream->write(21);
}

void kproject_dwm1000::set_anchor2_position(int anchor_id, int x, int y)
{
  _stream->write(20);
  _stream->write(9);
  _stream->write(anchor_id);
  _stream->write(x);
  _stream->write(x >> 8);
  _stream->write(y);
  _stream->write(y >> 8);
  _stream->write(21);
}

void kproject_dwm1000::set_anchor3_position(int anchor_id, int x, int y)
{
  _stream->write(20);
  _stream->write(10);
  _stream->write(anchor_id);
  _stream->write(x);
  _stream->write(x >> 8);
  _stream->write(y);
  _stream->write(y >> 8);
  _stream->write(21);
}

void kproject_dwm1000::_get_position(int tag_id, int &dist1, int &dist2, int &dist3, int &x, int &y)
{
  while (_stream->available()) _stream->read();
  _stream->write(20);
  _stream->write(12);
  _stream->write(tag_id);
  _stream->write(21);

  unsigned long start_millis = millis();
  while (_stream->available() < 14)
  {
    if ( ( millis() - start_millis ) > time_out )
    {
      x = -10000;
      y = -10000;
      return ;
    }
  }

  if ( _stream->read() != 20 ) return ;
  if ( _stream->read() != 7 ) return ;

  int recv_tag_id = _stream->read();
  int recv_x, recv_y;
  int distance1 , distance2, distance3;
  distance1 = _stream->read();
  distance1 = distance1 | (_stream->read() << 8);
  distance2 = _stream->read();
  distance2 = distance2 | (_stream->read() << 8);
  distance3 = _stream->read();
  distance3 = distance3 | (_stream->read() << 8);

  recv_x = _stream->read();
  recv_x = recv_x | (_stream->read() << 8);
  recv_y = _stream->read();
  recv_y = recv_y | (_stream->read() << 8);

  if ( _stream->read() != 21 ) return ;
  dist1 = distance1;
  dist2 = distance2;
  dist3 = distance3;
  x = recv_x;
  y = recv_y;
}

bool kproject_dwm1000::get_position(int tag_id, int &dist1, int &dist2, int &dist3, int &x, int &y)
{
  for ( int i = 0; i < RETRY_NO; i++)
  {
    _get_position(tag_id, dist1, dist2, dist3, x, y);
    if ( ( x != -10000 ) && ( y != -10000 ) )
    {
      return true;
    }

  }
  return false;
}
