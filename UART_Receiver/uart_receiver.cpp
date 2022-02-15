// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>



// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
int serial_port;

// Create new termios struc, we call it 'tty' for convention
struct termios tty;



static char read_buf [1];


char read_buf_distance[1] ;
// Get Num of Person
int Receive_Num_of_Person()
{

    char read_buf[1] ;
    int num_of_person = 0;
    
    while(read(serial_port, &read_buf, sizeof(read_buf)) && read_buf[0] != 'D' && read_buf[0] != 'E')
    {
        
        num_of_person = num_of_person*10 + (read_buf[0] - '0');
      
    }
    
    read_buf_distance[0] = read_buf[0];
    return num_of_person;
    
}




// Reading frame data and checksum from the serial port
void Get_Data_Checksum( float* frame_data)//, int* checksum_host)
{

    char read_buf[1];
    
    int index = 0;
    
    if(read_buf_distance[0] == 'E')
    {
         printf("\n DISTANCE : NOT AVAILABLE" );
         return;
    }    
  
    while(read(serial_port, &read_buf, sizeof(read_buf)) && read_buf[0] != 'E')
    {
    
            
        float index_data = 0;
        
        while(1)
        {
        
           
            index_data = index_data*10 + (read_buf[0] - '0');
            
            if(read(serial_port, &read_buf, sizeof(read_buf)) && (read_buf[0] == 'D' || read_buf[0] == 'E') )
            break;
        }
        
        
        
        frame_data[index] = index_data;
        
        printf("\n DISTANCE : %d cm", (int)index_data);
        index++;
        
        
        if(read_buf[0] == 'E')
        break;
        
    } 

}




int main(int argc, char* argv[]) 
{

  serial_port = open("/dev/ttyUSB0", O_RDWR);
  
  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 5;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0.01;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  

  // Allocate memory for read buffer, set size according to your needs
  

  
  
  bool depth_enable;
  depth_enable  = (argc > 1) ? true : false ;

  
  
  while(1)
  {
      //printf("\nWaiting..");
  
      memset(&read_buf, '\0', sizeof(read_buf));


      
      int num_of_person = 0;
    
        

      int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
        if (num_bytes < 0) 
        {
              printf("Error reading: %s", strerror(errno));
              return 1;
        } 


        
        if(read_buf[0] == 'S')
        {
            printf("\n[INFO] START OF NEW FRAME");
            
            num_of_person = Receive_Num_of_Person();
            
            printf("\nNum of person detected : %d", num_of_person);
            
            
            if(depth_enable)
            {
                // Array to store frame data  
                float dist_data[num_of_person];
                Get_Data_Checksum(dist_data);
                 
            }               
                
             printf("\n[INFO] END OF THE FRAME\n\n"); 
             
        }

   }
   

  close(serial_port);
  return 0; // success
}
