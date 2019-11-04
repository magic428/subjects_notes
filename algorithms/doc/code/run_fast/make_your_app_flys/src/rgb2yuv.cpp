




void calc_lum(RGB in, RGB out)
{
    int i;
    for(i = 0; i < IMGSIZE; i++)
    {
        double r, g, b, y;
        unsigned char yy;

        r = in[i].r;
        g = in[i].g;
        b = in[i].b;
        
        y = 0.299 * r + 0.587 * g + 0.114 * b;
        yy = y;
        out[i] = yy;
    }
}