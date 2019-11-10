#include <iostream>
#include <ctime>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>    

#include <boost/thread/thread.hpp>
#include <boost/thread/thread_time.hpp>    

using namespace std;
#define BOOST_DATE_TIME_SOURCE    


void time_elapsed_boost()
{
    double sum = 1.0;

    boost::posix_time::ptime start_;
    boost::posix_time::ptime stop_;

    // 获取当前时间
    start_ = boost::posix_time::microsec_clock::local_time();

    for (size_t i = 0; i < 1e7; i++)
        for (size_t i = 0; i < 1e3; i++)
            sum += 2.0/1.5;

    stop_ = boost::posix_time::microsec_clock::local_time();

    // 返回毫秒数
    float elapsed_milliseconds_ = (stop_ - start_).total_milliseconds();

    // 返回微秒数
    float elapsed_microseconds_ = (stop_ - start_).total_microseconds();

    cout << "boost takes " << elapsed_milliseconds_ << " ms"<< endl;
    cout << "boost takes " << elapsed_microseconds_ << " μs"<< endl;

}

#define CLOCKS_PER_MILSEC (CLOCKS_PER_SEC/1000)

void time_elapsed_buildin()
{ 
    time_t start_;
    time_t stop_;

    double sum = 1.0;
 
    // 获取当前时间
    start_ = clock();

    for (size_t i = 0; i < 1e7; i++)
        for (size_t i = 0; i < 1e3; i++)
            sum += 2.0/1.5;

    stop_ = clock();

    // 返回毫秒数
    double elapsed_milliseconds_ = (double )(stop_ - start_) / CLOCKS_PER_MILSEC;

    cout << "buildin takes " << elapsed_milliseconds_ << " ms"<< endl; 

}

void get_date_time()
{
    std::string str_time = boost::gregorian::to_iso_string(
                                    boost::gregorian::day_clock::local_day());    
        
    std::cout << str_time.c_str() << std::endl;  

            
    str_time = boost::posix_time::to_iso_string(    
                                boost::posix_time::second_clock::local_time());    
        
    // str_time 里存放时间的格式是YYYYMMDDTHHMMSS，日期和时间用大写字母T隔开了    
    std::cout << str_time.c_str() << std::endl;   
        
    int pos = str_time.find('T');    
    str_time.replace(pos, 1, std::string("-"));    
    str_time.replace(pos + 3, 0, std::string(":"));    
    str_time.replace(pos + 6, 0, std::string(":"));    
        
    std::cout << str_time.c_str() << std::endl;   
}

int main(int argc, char const *argv[])
{
    for(int i = 0; i < 20; i++){
        
        if (i % 2== 0){

            time_elapsed_buildin();
            time_elapsed_boost();
        }
        else {

            time_elapsed_boost();
            time_elapsed_buildin();
        }

        std::cout << std::endl;
        boost::thread::sleep(boost::get_system_time() + boost::posix_time::seconds(1));
    }
    
    get_date_time();
    
    return 0;
}