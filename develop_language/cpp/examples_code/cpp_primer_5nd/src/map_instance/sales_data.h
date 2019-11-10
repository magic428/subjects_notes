#include <iostream>
#include <string>

struct Sales_data {
    std::string isbn() const {return bookNo; };
    Sales_data & combine(const Sales_data &);

    double average_price() const;

    std::string bookNo;
    unsigned int units_sold = 0;
    double revenue = 0.0;
};