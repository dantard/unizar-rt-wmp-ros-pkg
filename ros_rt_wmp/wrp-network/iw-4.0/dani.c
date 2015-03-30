#include <stdio.h>
int qmain(int, char**);

int main(int argc, char**argv){
  
  int a = qmain(argc,argv);
  
  fprintf(stderr,"Return code:%d\n",a);
  return a;
}