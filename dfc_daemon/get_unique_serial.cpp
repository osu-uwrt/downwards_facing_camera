#include <dirent.h>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <unistd.h>

uint64_t discover_serial_number(void) {
    std::string netInfoDir = "/sys/class/net/";

    DIR *dp;
    struct dirent64 *ep;
    dp = opendir(netInfoDir.c_str());
    if (dp == NULL) {
        perror("Couldn't open the directory");
        return 0;
    }

    while ((ep = readdir64(dp)) != NULL) {
        if (ep->d_type == DT_LNK) {
            std::string interfacePath = netInfoDir + ep->d_name;
            char *actualPathCstr = realpath(interfacePath.c_str(), NULL);
            if (actualPathCstr) {
                std::string actualPath = actualPathCstr;
                free(actualPathCstr);

                printf("Actual Path: '%s'\n", actualPath.c_str());
            }
            else {
                perror("realpath");
            }
        }
    }

    if (closedir(dp)) {
        perror("closedir");
    }
    return 0;
}

int main() {
    printf("Discovered Serial Number: %016lX\n", discover_serial_number());
    return 0;
}
