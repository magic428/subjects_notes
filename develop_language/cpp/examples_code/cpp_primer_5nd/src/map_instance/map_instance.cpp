#include <map>
#include <set>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>  
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include "sales_data.h"


/***
 *  输出流文件结束标志 Ctrl + D
 */
using namespace std;


bool compareIsbn(const Sales_data &lhs, const Sales_data & rhs)
{
    return lhs.isbn() < rhs.isbn();
}

int map_test()
{
    map<string, int> word_count;
    string word;
    string word_origin;

    while(cin >> word){
        word_origin = word;
        // 全部转换为小写
        transform(word.begin(), word.end(), word.begin(), (int (*)(int))tolower);

        ++word_count[word];
    }

    // 使用基于范围的 for 语句遍历 map
    for(const auto & w : word_count){  
        cout << w.first << " occurs " << w.second <<
        (w.second > 1 ? " times" : "time") << endl;
    }

    map<string, size_t> word_cot;
    map<string, string> authors = { {"Joyce", "James"},
                                    {"Austen", "Jane"},
                                    {"Dickens", "Charles"}
    };

    vector<int> ivec;
    for (vector<int>::size_type i = 0; i <= 10; ++i){
        ivec.push_back(i);
        ivec.push_back(i);
    }

    set<int> iset(ivec.begin(), ivec.end());
    multiset<int> imset(ivec.begin(), ivec.end());
    auto iter_begin = iset.begin();
    iter_begin++;
    iset.erase(iter_begin, iset.end());
    cout << "ivec: " << ivec.size() << endl;
    cout << "iset: " << iset.size() << endl;
    cout << "imset: " << imset.size() << endl;

    /// 
    typedef bool (*isbn_type) (const Sales_data &lhs, const Sales_data & rhs);
    // multiset<Sales_data, decltype(compareIsbn) *> bookStore(compareIsbn); 
    multiset<Sales_data, isbn_type> bookStore(compareIsbn); 

    return 0;
}


map<string, string> build_map(ifstream &map_file)
{
    map<string, string> trans_map;
    string key, value;

    while(map_file >> key && getline(map_file, value)){
        // cout << key << endl;
        
        if( value.size() > 1) // 检查是否有转换规则   
            trans_map[key] = value.substr(1);
        else
            throw runtime_error("no rule(s) for " + key);
    }

    return trans_map;
}

const string transform(string &s, map<string, string> &t_map )
{
    auto map_it = t_map.find(s);

    if ( map_it != t_map.cend() )
        return map_it->second;
    else
        return s;   
}


void word_tansform(ifstream &map_file, ifstream &input)
{
    auto trans_map = build_map(map_file);
    string text; 

    // 以行为单位处理文件中
    while(getline(input, text)){
        istringstream stream(text);

        string word;
        bool first_word = true;

        // 处理每一行语句
        while(stream >> word){

            if(first_word)
                first_word = false;
            else
                cout << " ";
                cout << transform(word, trans_map);
        }
        cout << endl;
    }
}


namespace klm{

void GlobalInit(int* pargc, char*** pargv) {
  // Google flags.
  ::google::ParseCommandLineFlags(pargc, pargv, true);
  // Google logging.
  ::google::InitGoogleLogging(*(pargv)[0]);
  // Provide a backtrace on segfault.
  ::google::InstallFailureSignalHandler();
}

} // end of namespace klm

DEFINE_string(gpus, "", " separated by ','. -gpu all to run all gpus");
DEFINE_int32(iterations, 0, "int number foro test");

int main(int argc, char *argv[])
{
    // :google::InitGoogleLogging(argv[0]);
    klm::GlobalInit(&argc, &argv);

    // 打印输出到 stderr (当使用 logging 时).
    FLAGS_alsologtostderr = 1;
    
    LOG(INFO) << "glog is running";
    

    ifstream map_file("/home/klm/work/gitwork/work_note/dev_tools/cpp/code/cpp_primer_5nd/map_instance/map.txt");
    ifstream input_file("/home/klm/work/gitwork/work_note/dev_tools/cpp/code/cpp_primer_5nd/map_instance/test.txt");
    word_tansform(map_file, input_file);

    LOG(INFO) << "word_tansform done ";

    vector<string> gpus;
    // string flags_gpus(FLAGS_gpus);
    if(0 == strcmp(FLAGS_gpus.c_str(), "all"))
        LOG(INFO) << "Using GPUS: 0, 1, 2, 3, 4, 5, 6, 7, 8";
    else {

        boost::split(gpus, FLAGS_gpus, boost::is_any_of(","));
    
        for(auto &gpu: gpus)
            cout << gpu << endl;
    }

    LOG(INFO) << FLAGS_iterations;
    
    return 0;
}
