#include <stdio.h>
struct person{
    char name[30];
    int age;
    float weight;
};

struct person p[3];

void main()
{
    FILE *fp = NULL;
    int i = 0;
    fp = fopen("test.bin", "rb");
    fread(p, sizeof(struct person), 2, fp);
    fclose(fp);
    
    for(i = 0; i < 2; i++){
        printf("%s, %d, %.4f\n", p[i].name, p[i].age, p[i].weight);
    }
}