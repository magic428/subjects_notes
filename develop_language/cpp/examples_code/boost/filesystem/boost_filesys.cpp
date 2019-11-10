#include <iostream>
#include <string>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include "boost/algorithm/string.hpp"

/**
 * With "-std=c++11" can result in:
 *   undefined reference to: 
 *   `boost::filesystem::detail::copy_file(
 *      boost::filesystem::path const&, boost::filesystem::path const&, 
 *      boost::filesystem::copy_option, boost::system::error_code*)'
 * 
 * Solutions:
 * https://codeyarns.com/2017/09/20/undefined-reference-to-boost-copy_file/
 * 
*/

using namespace std;

int main_(int argc, char* argv[])
{
    string curPath = "/home/klm/work/test"; 
    boost::system::error_code ec;

    //定义一个可以递归的目录迭代器,用于遍历
    boost::filesystem::recursive_directory_iterator itEnd;
    for(boost::filesystem::recursive_directory_iterator itor( curPath.c_str() ); itor != itEnd ;++itor)
    {
        //itor->path().string() 是目录下文件的路径
        /**
         * 当curPath是相对路径时，itor->string()也是相对路径
         * 即当curPath = "../cur/",下面将输出"../cur/build.sh"
         */
        // 当curPath是绝对路径时，itor->string() 也是绝对路径
        string file = itor->path().string() ; // "/home/test/cur/build.sh"

        // 构造文件路径，以获得文件丰富的操作
        // path可以接受C风格字符串和string类型作为构造函数的参数，而提供的路径可以是相对路径，也可以是绝对路径。 
        boost::filesystem::path filePath(file);

        //path的方法如filename()等，返回的对象仍是path，如果可以通过path的string()方法，获取对象的string类型
        //parent_path()获得的是当前文件的父路径
        cout << filePath.parent_path() << endl;  // "/home/test/cur/"

        //filename()获得的是文件名，含拓展名
        cout << filePath.filename() << endl;  // "build.sh"
        cout << filePath.filename().string() << endl;

        //stem()获得的是文件的净文件名，即不含拓展名
        cout << filePath.stem() << endl; // "build"

        //extension()文件的拓展名（主要是".sh"而不是"sh"）
        cout << filePath.extension() << endl; // ".sh"

        // 获得文件的大小,单位为字节
        int nFileSize = boost::filesystem::file_size(filePath);
        //最后一次修改文件的时间
        //last_write_time()返回的是最后一次文件修改的绝对秒数
        //last_write_time(filePath,time(NULL))还可以修改文件的最后修改时间，相当于Linux中命令的touch
        if(boost::filesystem::last_write_time(filePath) - time(NULL) > 5)
        {
            /*
            *在工程实践中，当需要不断的扫目录，而目录又会不断的加入新文件时，
            *借助last_write_time()可以判断新入文件的完整性，以避免错误的处理还未写完的文件
            */
        }

        //判断文件的状态信息
        if(boost::filesystem::is_regular_file(file))
        {
            //is_regular_file(file)普通文件
            //is_directory(file)目录文件，如当遍历到"/home/test/cur/src/"时，这就是一个目录文件
            //is_symlink(file)链接文件
            cout << "regular_file" << endl; // ".sh"
        }

        //更改拓展名
        boost::filesystem::path tmpPath = filePath;
        //假设遍历到了cpp文件，想看下对应的.o文件是否存在
        tmpPath.replace_extension(".bash");
        //判断文件是否存在
        // if( boost::filesystem::exists( tmpPath.string() ) ) {

        //     //删除文件
        //     //remove只能删除普通文件，而不能删除目录
        //     cout << "Remove: " << tmpPath << endl; // ".o"
        //     boost::filesystem::remove(tmpPath.string());
        //     //remove_all则提供了递归删除的功能，可以删除目录
        //     boost::filesystem::remove_all(tmpPath.string());
        // }
        // 移动文件 & 拷贝文件
        // srcPath 原路径，srcPath 的类型为 string
        // destPath 目标路径，destPath 的类型为 string
        boost::filesystem::rename(filePath , tmpPath);
        boost::filesystem::copy_file(tmpPath, filePath, boost::filesystem::copy_option::overwrite_if_exists, ec);

    }

    // boost::filesystem 还可以创建目录： 
    boost::filesystem::path strFilePath("/home/klm/work/test/build/");
    if( !boost::filesystem::exists( strFilePath ) )
    {
        cout << "create_directories..." << endl; // ".sh"
        boost::filesystem::create_directories(strFilePath);

        // 拷贝目录: copy_directory
    }

    // copy_directory 的含义其实是创建目录. 拷贝的是目录属性相关信息
    cout << "copy_directory..." << endl; // ".sh"
    boost::filesystem::copy_directory("/home/klm/work/test","/home/klm/work/build/");

}

// 复制目录
bool CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir)
{
	boost::filesystem::recursive_directory_iterator end; //设置遍历结束标志，用recursive_directory_iterator即可循环的遍历目录
	boost::system::error_code ec;
	for (boost::filesystem::recursive_directory_iterator pos(strSourceDir); pos != end; ++pos)
	{
        //过滤掉目录和子目录为空的情况
		if(boost::filesystem::is_directory(*pos))
			continue;
		std::string strAppPath = boost::filesystem::path(*pos).string();
		std::string strRestorePath;
        
        //replace_first_copy在algorithm/string头文件中，在strAppPath中查找strSourceDir字符串，找到则用strDestDir替换，替换后的字符串保存在一个输出迭代器中
		boost::replace_first_copy(std::back_inserter(strRestorePath), strAppPath, strSourceDir, strDestDir);
		if(!boost::filesystem::exists(boost::filesystem::path(strRestorePath).parent_path()))
		{
			boost::filesystem::create_directories(boost::filesystem::path(strRestorePath).parent_path(), ec);
		}
		boost::filesystem::copy_file(strAppPath, strRestorePath, boost::filesystem::copy_option::overwrite_if_exists, ec);
	}
	if(ec)
	{
		return false;
	}
	return true;
}

/**
 * Copy regular files in a directory
*/
int main(int argc, char* argv[])
{
    string curPath = "/home/klm/work/test"; 
    string filename = "";
    boost::system::error_code ec;

    //定义一个可以递归的目录迭代器,用于遍历
    boost::filesystem::recursive_directory_iterator itEnd;
    for(boost::filesystem::recursive_directory_iterator itor( curPath.c_str() ); itor != itEnd ;++itor)
    {
        if(boost::filesystem::is_directory(itor->path()))
			continue;

        boost::filesystem::path filePath = itor->path();
        filename = filePath.filename().string();
       
        // boost::filesystem 还可以创建目录： 
        boost::filesystem::path dstPath("/home/klm/work/test_dst/"+filename);
        if( !boost::filesystem::exists( dstPath.parent_path() ) )
        {
            cout << "create_directories..." << endl;  
            boost::filesystem::create_directories(dstPath.parent_path());
        }
        boost::filesystem::copy_file(filePath, dstPath, boost::filesystem::copy_option::overwrite_if_exists, ec);
        cout << "copy file: " << dstPath.filename().string() << endl;  
    }
}