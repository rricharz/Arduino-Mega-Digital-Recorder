// ***************************************************************
// String.ino
//
// rricharz 2016
// ***************************************************************



//////////////////////////////////////////////////////////
void floatToStr(float value, char *res, int maxafterpoint)
//////////////////////////////////////////////////////////
// displays 4 significant digits, no negative numbers!
{
  float n;
  int afterpoint;
  
  if (abs(value) >= 1000) {
    afterpoint = 0;
    n = value + 0.5;       // rounding
  }
  else if (abs(value) >= 100) {
    afterpoint = 1;
    n = value + 0.05;      // rounding
  }
  else if (abs(value) >= 10) {
    afterpoint = 2;
    n = value + 0.005;     // rounding
  }
  else {
    afterpoint = 3;
    n = value + 0.0005;    // rounding
  }
  if (afterpoint > maxafterpoint)
    afterpoint = maxafterpoint;
  // Extract integer part
  int ipart = (int)n; 
  // Extract floating part
  float fpart = n - (float)ipart; 
  // convert integer part to string
  int i = intToStr(ipart, res, 1); 
  // check for display option after point
  if (afterpoint != 0) {
    res[i] = '.';  // add dot
 
    // Get the value of fraction part upto given no.
    // of points after dot. The third parameter is needed
    // to handle cases like 233.007
    fpart = fpart * pow(10, afterpoint); 
    intToStr((int)fpart, res + i + 1, afterpoint);
  }
}

//////////////////////////////////////
int intToStr(int x, char str[], int d)
//////////////////////////////////////
 // d is the number of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.

{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
 
    reverse(str, i);
    str[i] = '\0';
    return i;
}

////////////////////////////////
void reverse(char *str, int len)
////////////////////////////////
// reverses a string 'str' of length 'len'
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}


