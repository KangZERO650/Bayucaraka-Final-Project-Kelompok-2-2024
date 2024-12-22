#include <Servo.h>
#define enb 8 // enabling 

// Z axis dan Claw Mechanism
Servo zAxis;
Servo claw;
#define zAxisPin 11
#define clawPin 10

/*
// X axis and Y axis and Z axis Mechanism (sesuai shield)
#define nemaxStep 2
#define nemaxDir 5
#define nemayStep 3
#define nemayDir 6
#define nemazStep 4
#define nemazDir 7
*/

// X axis and Y axis and Z axis Mechanism (penyesuaian)
#define nemaxStep 2
#define nemaxDir 5
#define nemayStep 4
#define nemayDir 7
#define nemazStep 3
#define nemazDir 6

// rumus //
/*
Resolusi Stepper Motor = Sesuai di Datasheet [untuk full step] atau Resolusi Stepper Motor / Microstepping^{-1} [untuk mode selain full step]
Step per Revolusi = Resolusi Stepper Motor / 360
Gerak Linear (translasi) per Revolusi (mm) =  ( Jumlah Gigi Pulley * Pitch Gigi ) / Step per Revolusi
Gerak Linear (translasi) per Revolusi (cm) = Gerak Linear per Langkah (mm) / 10
Gerak Linear (translasi) per Langkah (mm) = Input Jarak (mm) / Gerak Linear (translasi) per Revolusi (mm) 
Gerak Linear (translasi) per Langkah (mm) = Input Jarak (cm) / Gerak Linear (translasi) per Revolusi (cm) 
Gerak Rotasi per Derajat (°) = ( Input Derajat Rotasi * Step per Revolusi ) /360
*/


// format inputan Serial //
/*

1. Untuk menggerakan X axis dan Y axis:

x<nilai pergerakan sumbu x>,y<nilai pergerakan sumbu y>
contoh:
x50,y150 -> sumbu x bergerak sejauh 50 satuan dan sumbu y bergerak sejauh 150 satuan

*note:
Jika ingin menggerakan salah satu sumbu, cukup isi sumbu lainnya dengan nilai 0 (nol)

2. Untuk menurunkan Z axis dan Mengatur Claw:

z<turun/naik>,c<jepit/buka>
Selanjutnya akan disingkat menggunakan huruf depan masing-masing kondisi!
z<t/n>,c<j/b>
contoh:
zt,cb -> Sumbu Z axis turun dan Claw membuka/melepas jepitan

*/

const int 
stepPerRevolution = 200*4, // fullstep * microstep^{-1} || fullstep * 1 [konfigurasi full step]
pulleyTeeth = 20, // jumlah gigi pulley
stepDelay = 300, // dalam microsecond

// batasan/constraint tiap penggerak //
xConstraint = 526, // Rumus inputan = Constraint Sebenarnya + 1 (mm)
yConstraint = 286, // Rumus inputan = Constraint Sebenarnya + 1 (mm)
zConstraint = 6650, // Rumus inputan = Constraint Sebenarnya + 1 (mm)
clawConstraint = 0; // Rumus inputan = Constraint Sebenarnya + 1 (mm)

int
prevKoorX = 0,
prevKoorY = 0;

const double 
gearPitch = 2.0, // pitch gigi
distancePerRevolution = ((double) pulleyTeeth * gearPitch) / stepPerRevolution; // pergerakan per 1 putaran penuh NEMA 17 (mm)

// inputan //
double 
x = 0.0,
y = 0.0;
char
zState = 'n', // zState 'n' = naik && zState 't' = turun, default nya adalah 'n' naik
cState = 'b'; // cState 'j' = jepit && cState 'b' = buka, default nya adalah 'b' buka

void parseInput(); // forward function parseInput()
void stepX(); // forward function stepX()
void stepY(); // forward function stepY()

void setup() {
    zAxis.attach(zAxisPin);
    claw.attach(clawPin);
    pinMode(nemaxStep, OUTPUT);
    pinMode(nemaxDir, OUTPUT);
    pinMode(nemayStep, OUTPUT);
    pinMode(nemayDir, OUTPUT);
    pinMode(nemazStep, OUTPUT);
    pinMode(nemazDir, OUTPUT);
    pinMode(enb, OUTPUT);
    claw.write(180); // claw membuka
    digitalWrite(enb, 0);
    //.write(0);
    //delay(22000);
    zAxis.write(90);
    Serial.begin(9600);

}

void loop(){
    if(Serial.available()> 0){
        String input = Serial.readStringUntil('\n');
        input.toLowerCase();
        parseInput(input);
    }
}

void parseInput(String inputan){
    // untuk X axis dan Y axis
    if(inputan.startsWith("x") && inputan.indexOf("y") != -1){
        // directionValue => searah jarum jam = 1 && berlawanan jarum jam = 0
        int 
        xStepper = 0, // Mengatur pergerakan nyata sumbu X Stepper Motor
        xStepperDir = 0, // Mengatur arah gerak nyata Sumbu X Stepper Motor
        yStepper = 0, // Mengatur pergerakan nyata sumbu Y Stepper Motor
        yStepperDir = 0; // Mengatur arah gerak nyata Sumbu Y Stepper Motor
        int xIndex = inputan.indexOf('x')+1;
        int yIndex = inputan.indexOf('y')+1;
        x = inputan.substring((xIndex),(inputan.indexOf(','))).toInt();
        y = inputan.substring(yIndex).toInt();
        // debug
        // Serial.print(x);
        // Serial.print("   ||   ");
        // Serial.println(y);

        // Sumbu X
        if(x<prevKoorX){
            xStepper = prevKoorX - x;
            xStepperDir = -1;
        } else if(x>prevKoorX){
            xStepper = x - prevKoorX;
            xStepperDir = 1;
        } else if(x==prevKoorX){
            xStepperDir = 0;
        } else {
            Serial.println("Arithmetic Logic Error in X Coordinate!");
        }
        // Kirim ke Stepper sumbu X
        if(x<xConstraint && x>=0){
            if(xStepperDir == 0){
                ;
            } else if(xStepperDir == -1){
                stepX(0,xStepper); // Berlawanan arah jarum jam
            } else if(xStepperDir == 1){ 
                stepX(1,xStepper); // Searah jarum jam
            } else {
                Serial.println("Direction Logic Error in X Coordinate!");
            }

        } else {
            Serial.println("Physical Logic Error in X Coordinate!");
            Serial.println("Please be Patient!");
        }

        // Sumbu Y
        if(y<prevKoorY){
            yStepper = prevKoorY - y;
            yStepperDir = -1;
        } else if(y>prevKoorY){
            yStepper = y - prevKoorY;
            yStepperDir = 1;
        } else if(y==prevKoorY){
            yStepperDir = 0;
        } else {
            Serial.println("Arithmetic Logic Error in Y Coordinate!");
        }
        // Kirim ke Stepper sumbu Y
        if(y<yConstraint && y>=0){
            if(yStepperDir == 0){
                ;
            } else if(yStepperDir == -1){
                stepY(0,yStepper); // Berlawanan arah jarum jam
            } else if(yStepperDir == 1){ 
                stepY(1,yStepper); // Searah jarum jam
            } else {
                Serial.println("Direction Logic Error in Y Coordinate!");
            }

            prevKoorX = x;
            prevKoorY = y;
        } else {
            Serial.println("Physical Logic Error in Y Coordinate!");
            Serial.println("Please be Patient!");
        }

    } 
    // untuk Z axis dan claw
    else if(inputan.startsWith("z") && inputan.indexOf("c") != -1 && inputan.length()<6){
        zState = inputan.substring(1,2)[0];
        cState = inputan.substring((inputan.indexOf("c"))+1,(inputan.indexOf("c")+2))[0];
        // debug
        // Serial.print(zState);
        // Serial.print("   ||   ");
        // Serial.println(cState);


        if(cState == 'j'){
            claw.write(60);
        } else if(cState == 'b'){
            claw.write(180);
        }


        if(zState == 'n') {
            //
            zAxis.write(0);
            delay(1000);
            zAxis.write(90);
        } else if(zState == 't'){
            //
            zAxis.write(180);
            delay(1000);
            zAxis.write(90);
        } else if(zState == 'm'){
           // Z Axis Diam
        }

    } else {
        Serial.println("Input error!");
        Serial.println("Please fill with correct format!");
    }
    //
}

void stepX(int dir, int step){
    //debug
    // Serial.print(dir);
    // Serial.print("  ||  ");
    // Serial.println(step);

    step = (int)(step / distancePerRevolution);

    // arah rotasi 
    digitalWrite(nemaxDir, dir);
    delayMicroseconds(10);

    // Sinyal Langkah
    for(int i =0;i<step;i++){
        digitalWrite(nemaxStep, 1);
        delayMicroseconds(stepDelay);
        digitalWrite(nemaxStep, 0);
        delayMicroseconds(stepDelay);
    }
}

void stepY(int dir, int step){
    //debug
    // Serial.print(dir);
    // Serial.print("  ||  ");
    // Serial.println(step);

    step = (int)(step / distancePerRevolution);

    // arah rotasi 
    digitalWrite(nemayDir, dir);
    delayMicroseconds(10);

    // Sinyal Langkah
    for(int i =0;i<step;i++){
        digitalWrite(nemayStep, 1);
        delayMicroseconds(stepDelay);
        digitalWrite(nemayStep, 0);
        delayMicroseconds(stepDelay);
    }
}