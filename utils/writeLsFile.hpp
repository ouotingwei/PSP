#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

struct Waypoint {
    double x;
    double y;
    double z;
    double W;
    double P;
    double R;
    double V;
    std::string C;
};

int writeLsFile(const std::string& file, const std::vector<Waypoint>& waypoints) {
    std::ofstream f(file);
    if (!f.is_open()) {
        std::cout << "Error opening file: " << file << std::endl;
        return -1;
    }

    f << "/PROG  " << file << "\n";
    f << "/ATTR\n";
    f << "OWNER       = MNEDITOR;\n";
    f << "COMMENT     = \"\";\n";
    f << "PROG_SIZE   = 636;\n";
    f << "CREATE      = DATE 23-01-07  TIME 11:59:14;\n";
    f << "MODIFIED    = DATE 23-01-07  TIME 12:02:18;\n";
    f << "FILE_NAME   = ;\n";
    f << "VERSION     = 0;\n";
    f << "LINE_COUNT  = 4;\n";
    f << "MEMORY_SIZE = 992;\n";
    f << "PROTECT     = READ_WRITE;\n";
    f << "TCD:  STACK_SIZE    = 0,\n";
    f << "      TASK_PRIORITY = 50,\n";
    f << "      TIME_SLICE    = 0,\n";
    f << "      BUSY_LAMP_OFF = 0,\n";
    f << "      ABORT_REQUEST = 0,\n";
    f << "      PAUSE_REQUEST = 0;\n";
    f << "DEFAULT_GROUP    = 1,*,*,*,*;\n";
    f << "CONTROL_CODE     = 00000000 00000000;\n";

    f << "/MN\n";
    f << "   1:J P[1] 100"<<"%"<<" FINE    ;\n";
    for (size_t i = 2; i <= waypoints.size(); ++i) {
        f << "   " << i << ":L P[" << i << "] " << waypoints[i - 1].V << "mm/sec " << waypoints[i - 1].C << "    ;\n";
    }

    f << "/POS\n";
    for (size_t i = 1; i <= waypoints.size(); ++i) {
        f << "P[" << i << "]{\n";
        f << "   GP1:\n";
        f << "    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',\n";
        f << "    X =  " << std::fixed << std::setprecision(3) << waypoints[i - 1].x << "  mm,    Y =   " << waypoints[i - 1].y << "  mm,    Z =   " << waypoints[i - 1].z << "  mm,\n";
        f << "    W =  " << waypoints[i - 1].W << " deg,    P =   " << waypoints[i - 1].P << " deg,    R =   " << waypoints[i - 1].R << " deg\n";
        f << "};\n";
    }

    f << "/END\n";
    f.close();
    return 0;
}

