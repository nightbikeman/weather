#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <curses.h>
  /* baudrate settings are defined in <asm/termbits.h>, which is
   *       included by <termios.h> */
#define BAUDRATE B9600
  /* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyS0"
#define _POSIX_SOURCE 1		/* POSIX compliant source */
#define FALSE 0
#define TRUE 1
volatile int STOP = FALSE;

#define EXP_PROMPT "Exp> "

#define NO_ELEMS(a) (sizeof(a)/sizeof((a)[0]))

typedef struct {
	int fd;
	struct termios oldtio;
} OBSERVER;

typedef struct {
	float   port_val;
	float reading;
} POINT;

typedef struct {
	POINT cal[2];
	int number;
	char *name;
	char *units;
} TEMP_PROBE;

typedef struct {
	POINT cal[2];
	int number;
	char *name;
	char *units;
} BARO_PROBE;

typedef struct {
	POINT cal;
	int number;
	char *name;
	char *units;
} HUM_PROBE;

typedef struct
{
	POINT curve[11];
} HUMIDITY_CURVE;

#define  Command_Observer(a,b) _Command_Observer(a,b,1)
#define  Command_Observer_nr(a,b) _Command_Observer(a,b,0)
int _Command_Observer(OBSERVER *ob,char *command,int response)
{
	char buf[255];
	char com[255];
	char exp[255];
	char exp1[255];
	int r;
	int res;
	strcpy(buf,"");
	strcpy(exp, command);

	/* construct and isuues the command */
	sprintf(com, "%s\r", command);
	strcpy(exp1,EXP_PROMPT);
	strcat(exp1,command);
	res = write(ob->fd,com,strlen(com));
	if (res != strlen(com)) printf("Send error\n");
	while(( strncmp(buf,exp,strlen(exp)) != 0) && (strncmp(buf, exp1, strlen(exp1)) != 0)) 
	{
		res = read(ob->fd, buf, 255);
		buf[res] = 0; /* set end of string, so we can printf */
	}
	if (response) {
		res = read(ob->fd, buf, 255);
		buf[res] = 0; /* set end of string, so we can printf */

		r = atoi(buf);
	} else
		r = -99999999;
	return r;
}


void Set_Digital(OBSERVER *ob,int port,char value)
{
		char buf[255];
		sprintf(buf,"D %d %d",port,value);
		Command_Observer_nr(ob,buf);
}
int Initialise_Observer(OBSERVER *ob,char *Port)
{
  int c, res;
  struct termios newtio;
  /* 
   *         Open modem device for reading and writing and not as controlling tty
   *                 because we don't want to get killed if linenoise sends CTRL-C.
   *                       */
  ob->fd = open (Port, O_RDWR | O_NOCTTY);
  if (ob->fd < 0)
    {
      perror (Port);
      return (-1);
    }

  tcgetattr (ob->fd, &ob->oldtio);	/* save current serial port settings */
  bzero (&newtio, sizeof (newtio));	/* clear struct for new port settings */

  /* 
   *         BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
   *                 CRTSCTS : output hardware flow control (only used if the cable has
   *                                   all necessary lines. See sect. 7 of Serial-HOWTO)
   *                                           CS8     : 8n1 (8bit,no parity,1 stopbit)
   *                                                   CLOCAL  : local connection, no modem contol
   *                                                           CREAD   : enable receiving characters
   *                                                                 */
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

  /*
   *         IGNPAR  : ignore bytes with parity errors
   *                 ICRNL   : map CR to NL (otherwise a CR input on the other computer
   *                                   will not terminate input)
   *                                           otherwise make device raw (no other input processing)
   *                                                 */
 /* newtio.c_iflag = IGNPAR | ICRNL; */
  newtio.c_iflag = IGNPAR ; 


  /*
   *        Raw output.
   *              */
  newtio.c_oflag = 0;

  /*
   *         ICANON  : enable canonical input
   *                 disable all echo functionality, and don't send signals to calling program
   *                       */
  newtio.c_lflag = ICANON;

  /* 
   *         initialize all control characters 
   *                 default values can be found in /usr/include/termios.h, and are given
   *                         in the comments, but we don't need them here
   *                               */
  newtio.c_cc[VINTR] = 0;	/* Ctrl-c */
  newtio.c_cc[VQUIT] = 0;	/* Ctrl-\ */
  newtio.c_cc[VERASE] = 0;	/* del */
  newtio.c_cc[VKILL] = 0;	/* @ */
  newtio.c_cc[VEOF] = 4;	/* Ctrl-d */
  newtio.c_cc[VTIME] = 0;	/* inter-character timer unused */
  newtio.c_cc[VMIN] = 1;	/* blocking read until 1 character arrives */
  newtio.c_cc[VSWTC] = 0;	/* '\0' */
  newtio.c_cc[VSTART] = 0;	/* Ctrl-q */
  newtio.c_cc[VSTOP] = 0;	/* Ctrl-s */
  newtio.c_cc[VSUSP] = 0;	/* Ctrl-z */
  newtio.c_cc[VEOL] = 0;	/* '\0' */
  newtio.c_cc[VREPRINT] = 0;	/* Ctrl-r */
  newtio.c_cc[VDISCARD] = 0;	/* Ctrl-u */
  newtio.c_cc[VWERASE] = 0;	/* Ctrl-w */
  newtio.c_cc[VLNEXT] = 0;	/* Ctrl-v */
  newtio.c_cc[VEOL2] = 0;	/* '\0' */

  /* 
   *         now clean the modem line and activate the settings for the port
   *               */
  tcflush (ob->fd, TCIFLUSH);
  tcsetattr (ob->fd, TCSANOW, &newtio);

  /* reset digiatl port A  */
  Set_Digital(ob,3,139);

  /* start counter (falling edge) for rain gauge */
  Command_Observer_nr(ob,"C 3 2");

  return 0;

}


void Close_Observer(OBSERVER *ob)
{
  /* restore the old port settings */
  tcsetattr (ob->fd, TCSANOW, &ob->oldtio);
  close(ob->fd);
}
int Read_Analog(OBSERVER *ob,int no)
{
		char buf[255];
		sprintf(buf,"A %d",no);
		return Command_Observer(ob,buf);
}
int Read_Timer(OBSERVER *ob,int no,int time, int reset)
{
		char buf[255];
		sprintf(buf,"C %d %d %d",no,time,reset);
		return Command_Observer(ob,buf);
}

float interpolate(POINT *A,POINT *B,int val)
{
	float m;
	float c;

	m=(A->reading-B->reading)/(A->port_val-B->port_val);

	c=A->reading-m*A->port_val;

	return m*val+c;

}
float Read_Pressure(OBSERVER *ob,BARO_PROBE *T)
{
	int port_val=Read_Analog(ob,T->number);

	return interpolate(&T->cal[0],&T->cal[1],port_val);

}
float Read_Temperature(OBSERVER *ob,TEMP_PROBE *T)
{
	int port_val=Read_Analog(ob,T->number);

	return interpolate(&T->cal[0],&T->cal[1],port_val);

}
float Read_Windspeed(OBSERVER *ob)
{
	int port_val=Command_Observer(ob,"C 0 8 6");

	return 1.55/(((float)port_val)/1000000.0);

}
float Read_Winddirection(OBSERVER *ob)
{
	float w1=Read_Analog(ob,6);
	float w2=Read_Analog(ob,7);
	float delta=700.0;
	float bearing;

  if ((w1 > 5120 - (2 * delta)) && (w1 < 4920) && (w2 < (2 * delta)) && (w2 > 200))
    delta = (9 * delta + (w2 + 5120 - w1) / 2) / 10;
  else if (((w2 > 5120 - (2 * delta)) && (w2 < 4920)) && (w1 < (2 * delta)) && (w1 > 200)) 
    delta = (9 * delta + (w1 + 5120 - w2) / 2) / 10 ;

  /* Decide if we have a valid bearing off of one of the wipers.*/
  if (((w1 > delta) && (w1 < (5120 - delta))) && ((w2 > (5120 - delta)) || (w2 < delta)))
    bearing = 180 * (w1 - delta) / (5120 - 2 * delta);
  else if (((w2 > delta) && (w2 < (5120 - delta))) && ((w1 > (5120 - delta)) || (w1 < delta))) 
    bearing = 180 + 180 * (w2 - delta) / (5120 - 2 * delta);
  


	return bearing;

}
float Read_Humidity(OBSERVER *ob,HUM_PROBE *T)
{
	int port_val;
	float rhumidity=-100.0;

	POINT curve[]={	/* percentage port val, relative humidity */
		            {100.0,100},
					{95.2,90},
					{91.4,80},
					{88.0,70},
					{85.0,60},
					{82.5,50},
					{80.3,40},
					{78.2,30},
					{76.4,20},
					{74.7,10},
					{72.7,0 }
				};


	/* enable the humidity detector */
	Set_Digital(ob,0,1);
	port_val=Read_Timer(ob,T->number,7,1);

	/* disable the humidity detector */
	Set_Digital(ob,0,0);

	{
		/* compute the percentage of the max of the portval */
		float perc_pv=port_val/T->cal.port_val*T->cal.reading;
		int c=0;
		while( (curve[c].port_val > perc_pv) && (c < NO_ELEMS(curve)) )
			c++;

		if (( c >= NO_ELEMS(curve)) || (c == 0 ))
			printw("Calibration Error Reading Humidity c=%d %lf\n",c,perc_pv);
		else
		{
			rhumidity=interpolate(&curve[c-1],&curve[c],perc_pv);
		}
	}

	return rhumidity;

}


main ()
{
	int v=0;
	int res=0;
	char buf[255];
	OBSERVER ob;
	TEMP_PROBE temperature[3]={
						{940,20.22,615,37.0,1,"Case","C"},
						{2100,0.0,580,31.0,4,"Inside","C"},
						{2100,0.0,660,31.0,5,"Outside","C"}
						};
	HUM_PROBE humidity[2]={
							{5460,100.0,1,"Inside","%Rh"},
							{5630,100.0,2,"Outside","%Rh"}
						};
	BARO_PROBE pressure= {1655,1001,3845,1038.0,0,"","mb"};
	initscr();

	if (Initialise_Observer(&ob,MODEMDEVICE) != 0)
	{
			fprintf(stderr,"Failed to initialise Observer\n");
			exit(1);
	}
	while (1)
	{
	move(0,0);
	printw("\nTemperature\n");
	for (v=0;v<3;v++)
	{
		printw("temperature %s = %.2f %s\n",temperature[v].name,Read_Temperature(&ob,&temperature[v]),temperature[v].units);
	}
	printw("\nHumidity\n");
	for (v=0;v<2;v++)
	{
		printw("Humidity %s = %.2f %s\n",humidity[v].name,Read_Humidity(&ob,&humidity[v]),humidity[v].units);
	}
	printw("\nPressure\n");
		printw("Pressure %.2f %s\n",Read_Pressure(&ob,&pressure),pressure.units);
	printw("\nWind\n");
		printw("Speed = %.2f Mph %d\n",Read_Windspeed(&ob), Command_Observer(&ob,"C 0 8 6"));
		printw("Direction = %.2f \n",Read_Winddirection(&ob));

	printw("\nAnalogs\n");
	for (v=0;v<8;v++)
	{
		printw("%2d = %6d \n",v,Read_Analog(&ob,v));
	}
#ifdef COUNTERS
	printw("\nTimers\n");
	for (v=0;v<4;v++)
	{
		printw("%2d = %6d \n",v,Read_Timer(&ob,v,0,0));
	}
#endif
	refresh();
	}
	endwin();
	Close_Observer(&ob);
}
