#include <LogStream.h>

LogStream::LogStream()
{
  buff_ = new char[128];
  buffp_ = buff_;
  logLength_ = 0;
}

size_t
LogStream::write(uint8_t c)
{
  *buffp_ = c;
  if ('\n' == c)
  {
    mediator_->sendLogMsg(buff_, logLength_);
    buffp_ = buff_;
    logLength_ = 0;
  }
  else
  {
    buffp_++;
    logLength_++;
  }
  return 1;
}
