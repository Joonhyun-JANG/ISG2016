#include<iostream>
#include<stdio.h>

int main(){
	FILE *fp;
	fp = fopen("./line_distance_front.txt", "w");
	for(int i=0;i<270;i++){
		if(i<=30) fprintf(fp, "%d %d\n", i, 25);
		else if(i>=210) fprintf(fp, "%d %d\n", i, 90);
		else fprintf(fp, "%d %d\n", i, 0);
	}

	fclose(fp);
	return 0;
}
