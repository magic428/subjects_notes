#include <iostream>

using namespace std;


class DefaultParamter
{
public:
    DefaultParamter() { }

    virtual void Solve(const char* resume_file = NULL);
};


void DefaultParamter::Solve(const char* resume_file) {
    cout << "Solve" << resume_file << endl;
}


int main()
{
    cout << "Hello World!" << endl;
    DefaultParamter *dp = new DefaultParamter();
    dp->Solve("he");
    return 0;
}

