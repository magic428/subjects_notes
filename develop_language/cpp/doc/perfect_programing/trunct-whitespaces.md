# C++ 去掉字符串中首尾空格和所有空格

## 去掉首尾空格

```cpp
void trim(string &s)
{
    if( !s.empty() )
    {
        s.erase(0,s.find_first_not_of(" "));
        s.erase(s.find_last_not_of(" ") + 1);
    }
}
```

## 去掉字符串中所有空格

```cpp
void trim(string &s)
{
    int index = 0;
    if( !s.empty())
    {
        while( (index = s.find(' ',index)) != string::npos)
        {
            s.erase(index,1);
        }
    }
}
```
