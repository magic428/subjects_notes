#ifndef _DLL_TEST_H_
#define _DLL_TEST_H_

#include <string>
#include <iostream>

#ifndef LIB_API
    #ifdef LIB_EXPORTS
        #if defined(_MSC_VER)
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __attribute__((visibility("default")))
        #endif
    #else
        #if defined(_MSC_VER)
            #define LIB_API __declspec(dllimport)
        #else
            #define LIB_API
        #endif
    #endif
#endif

LIB_API int sum(int a, int b);

class Detector {
public:
    const int uCurGpuId_;

    static LIB_API Detector* createNew(std::string& cfg_name, 
                      int width, 
                      int height, 
                      int depth, 
                      int gpu_id = 0)
    {
        std::cout << "Detector createNew" << std::endl;
        return new  Detector(cfg_name, width, height, depth, gpu_id);
    }

    Detector( std::string& cfg_name, 
                      int width, 
                      int height, 
                      int depth, 
                      int gpu_id = 0) 
            : sCfgName_(cfg_name),
              nWidth_(width), 
              nHeight_(height), 
              nDepth_(depth),
              uCurGpuId_(gpu_id)
    {
        std::cout << "Detector Constructor" << std::endl;
    }
    
	LIB_API ~Detector() { } 

    LIB_API int get_net_width() const { return nWidth_; }
    LIB_API int get_net_height() const { return nHeight_; }
    LIB_API int get_net_color_depth() const { return nDepth_; }
    LIB_API std::string& get_cuda_context() { return sCfgName_; }
    LIB_API static int maxSize;

private:
    std::string sCfgName_;
    int nWidth_;
    int nHeight_;
    int nDepth_;
};

#endif