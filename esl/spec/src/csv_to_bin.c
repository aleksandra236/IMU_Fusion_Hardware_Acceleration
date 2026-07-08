/**
 * @file csv_to_bin.c
 * @brief Converts CSV sensor data to binary format.
 * 
 * Expected CSV format:
 *   time,gx,gy,gz,ax,ay,az
 *   0.01,1.23,4.56,7.89,0.01,0.02,1.00
 *   ...
 * 
 * USAGE:
 *   gcc -o csv_to_bin csv_to_bin.c
 *   ./csv_to_bin sensor_data.csv sensor_data.bin
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    float time;
    float gx, gy, gz;
    float ax, ay, az;
} SensorSample;

int main(int argc, char* argv[]) {
    
    if (argc < 3) {
        printf("Usage: %s <input.csv> <output.bin>\n", argv[0]);
        return 1;
    }
    
    const char* csvPath = argv[1];
    const char* binPath = argv[2];
    
    FILE* csvFile = fopen(csvPath, "r");
    if (!csvFile) {
        printf("Error: Cannot open %s\n", csvPath);
        return 1;
    }
    
    FILE* binFile = fopen(binPath, "wb");
    if (!binFile) {
        printf("Error: Cannot create %s\n", binPath);
        fclose(csvFile);
        return 1;
    }
    
    char line[512];
    int count = 0;
    int lineNum = 0;
    
    while (fgets(line, sizeof(line), csvFile)) {
        lineNum++;
        
        // Skip header
        if (lineNum == 1) {
            // Check if it's a header
            if (strstr(line, "time") != NULL || strstr(line, "gx") != NULL) {
                printf("Skipped header: %s", line);
                continue;
            }
        }
        
        SensorSample sample;
        int parsed = sscanf(line, "%f,%f,%f,%f,%f,%f,%f",
                            &sample.time,
                            &sample.gx, &sample.gy, &sample.gz,
                            &sample.ax, &sample.ay, &sample.az);
        
        if (parsed == 7) {
            fwrite(&sample, sizeof(SensorSample), 1, binFile);
            count++;
        } else {
            printf("Warning: Line %d has %d fields (expected 7)\n", lineNum, parsed);
        }
    }
    
    fclose(csvFile);
    fclose(binFile);
    
    printf("Converted %d samples\n", count);
    printf("Input:  %s\n", csvPath);
    printf("Output: %s\n", binPath);
    printf("Size: %lu bytes\n", (unsigned long)(count * sizeof(SensorSample)));
    
    return 0;
}
