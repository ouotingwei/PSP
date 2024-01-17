#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void ftp_transfer(const char *host_address, const char *file_directory) {
    char command[256];
    FILE *ftp_script;

    // Create a temporary FTP script file
    ftp_script = fopen("ftp_script.txt", "w");
    if (ftp_script == NULL) {
        printf("Failed to create FTP script file.\n");
        return;
    }

    // Write FTP commands to the script file
    fprintf(ftp_script, "user anonymous\n");
    fprintf(ftp_script, "binary\n");
    fprintf(ftp_script, "put %s\n", file_directory);
    fprintf(ftp_script, "quit\n");

    fclose(ftp_script);

    // Execute FTP command with the script file
    snprintf(command, sizeof(command), "ftp -n %s < ftp_script.txt", host_address);
    int status = system(command);

    // Remove the temporary script file
    remove("ftp_script.txt");

    if (status == -1) {
        printf("Failed to execute FTP command.\n");
    } else {
        printf("FTP transfer completed.\n");
    }
}

int main(int argc, char *argv[]) {
    const char *host_address = "127.0.0.1";
    const char *file_directory = "D:\\Download\\B001.LS";

    ftp_transfer(host_address, file_directory);

    return 0;
}

