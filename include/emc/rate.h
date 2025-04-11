#ifndef EMC_SYSTEM_RATE_H_
#define EMC_SYSTEM_RATE_H_

class RateImpl;


namespace emc
{

class Rate
{
public:
    Rate(double freq);
    ~Rate();
    void sleep();

private:
    RateImpl* rate_;
};

} // end namespace emc

#endif
