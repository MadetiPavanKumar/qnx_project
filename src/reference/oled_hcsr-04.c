#include <stdio.h>
#include<stdlib.h>
#include<pthread.h>
#include<unistd.h>
#include <sys/neutrino.h>
#include<errno.h>
#include "oled.h"
#include "hc_sr04.h"
#define DONE_EVENT 1
#define TRIG1 17
#define ECHO1 27
#define TRIG2 22
#define ECHO2 23
int chid = 0;
typedef struct{
	float sensor1;
	float sensor2;
}sensor_t;
typedef union{
	sensor_t msg;
	struct _pulse pulse;
}rceive_t;

void *display_thread(void *arg){
	rceive_t rcv;
	if(oled_init("/dev/i2c1")!=0){
		return NULL;
	}
	oled_clear();
	oled_set_cursor(0,0);
	oled_print("QNX SAFETY SUPERVISOR");
	chid = ChannelCreate(0);
	if(chid==-1){
		printf("channelCreation fialed %d\n",chid);
		return NULL;
	}
	printf("channelCreation success %d\n",chid);
	while(1){
		int rcvid = MsgReceive(chid,&rcv,sizeof(rcv),NULL);
		if(rcvid==-1){
			printf("rcvid failed %d\n",rcvid);
			return NULL;
		}
		if(rcvid==0){
			printf("msgPulse received\n");
			if(rcv.pulse.code==DONE_EVENT){
				printf("sensor done event received closing\n");
				break;
			}
		}else{
			float dist1 = rcv.msg.sensor1;
			float dist2 = rcv.msg.sensor2;
			oled_set_cursor(0,2);
			oled_print("DIST1: ");
			oled_print_float(dist1,1);
			oled_set_cursor(0,4);
			oled_print("DIST2: ");
			oled_print_float(dist2,1);
			MsgReply(rcvid,EOK,NULL,0);
		}
	}
//	oled_clear();
	ChannelDestroy(chid);
	return NULL;
}
void *sensor_thread(void *arg){
//	sensor_t sensor;
	rceive_t rcv;
	int sensor1 =hc_sr04_begin(TRIG1,ECHO1);
	int sensor2 = hc_sr04_begin(TRIG2,ECHO2);
	if(sensor1 < 0 || sensor2<0){
		return NULL;
	}int count = 0 ;
	while(chid==0){
		usleep(1000);
	}
	int coid =  ConnectAttach(0,0,chid,_NTO_SIDE_CHANNEL,0);
	if(coid==-1){
		printf("connectAttach failed %d\n",coid);
		return NULL;
	}
	printf("connection success %d\n",coid);
	int reply=0;
	while(1){
		rcv.msg.sensor1=read_distance(sensor1);
		rcv.msg.sensor2=read_distance(sensor2);
		printf("distance = %f\n",rcv.msg.sensor2);
		MsgSend(coid,&rcv,sizeof(rcv),&reply,sizeof(reply));
//		printf("[sensor] msg sent and reply received count = %d\n",count);
		count++;
		if(count==5){
			MsgSendPulse(coid,10,DONE_EVENT,10);
			break;
		}
	}
	ConnectDetach(coid);
	return NULL;
}
int main(void){
	pthread_t displaytid;
	pthread_t sensortid;
	if(pthread_create(&displaytid,NULL,display_thread,NULL)!=0){
		perror("pthread_create");
		return EXIT_FAILURE;
	}
	if(pthread_create(&sensortid,NULL,sensor_thread,NULL)!=0){
		perror("pthread_create sensor");
		return EXIT_FAILURE;
	}
	pthread_join(displaytid,NULL);
	pthread_join(sensortid,NULL);
	printf("program completed\n");
	return EXIT_SUCCESS;
}
