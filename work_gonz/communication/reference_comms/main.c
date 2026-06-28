#include<stdio.h>
#include<string.h>
#include <libpynq.h>

//change size
void to_xy (int* x, int* y, uint8_t* c)
{
  int h = 100;
  for(int i = 0; i < 3; i++)
  {
    *x += h * (c[i] - 48);
    h /= 10;
  }
  h = 100;
   for(int i = 0; i < 3; i++)
  {
    *y += h * (c[i+3] - 48);
    h /= 10;
  }
}

//change size
void coord_to_string(int x, int y, int obj, int colour, uint8_t *str)
{
  str[6] = 48 + obj;
  str[7] = 48 + colour;
  for(int i = 0; i < 3; i++)
  {
    str[2-i] = (uint8_t)(x % 10) + 48;
    str[5-i] = (uint8_t)(y % 10) + 48;
    y /= 10;
    x = x/10;
  }
}

int main(void) {
  pynq_init();
  switchbox_set_pin(IO_AR0 ,SWB_UART0_RX);
  switchbox_set_pin(IO_AR1 ,SWB_UART0_TX);
  uart_init(UART0);
  uart_reset_fifos(UART0); 

  //change size
  //char e[5][9] = {"12313430", "50050031", "70070030", "30030044", "60030045"}; //used for send
  int x9 = 123;
  int y9 = 123;
  int obj = 2;
  int colour = 1;
  uint8_t size[4], c[9]; //used for recv //change size

  while(1){
    //recv
    int j = 0;
    int x10 = 0;
    int y10 = 0;
    printf("loop\n");
    if(uart_has_data(UART0))
    {
      for(int i = 0; i < 4; i++)
        size[i] = uart_recv(UART0);

      printf("strlen1: %d\n", size[0]);
      for(int i = 0; i < size[0]; i++)
        c[i] = uart_recv(UART0);

      j = 1;
    }
    if(j != 0)
    {
      to_xy(&x10, &y10, c);
      c[size[0]] = '\0';
      printf("data: %s\n", c);
    }
    printf("x: %d\n", x10);
    printf("y: %d\n", y10);
    

    //code

    //send //change size
    coord_to_string(x9, y9, obj, colour, c);
    uint8_t d = 8;
    printf("loop\n");
    uart_send(UART0, d);
    d = 0;
    for(int i = 0; i < 3; i++)
    {
      uart_send(UART0, d);
    }
    d = 8;
    for(int i = 0; i < d; i++)
    {
      uart_send(UART0, c[i]);
      printf("%c",c[i]);
    }
    printf("\n");
    sleep_msec(3000);
    x9++;
  }

  printf("The end");
    
  uart_destroy(UART0);
  pynq_destroy();
  return EXIT_SUCCESS;
}