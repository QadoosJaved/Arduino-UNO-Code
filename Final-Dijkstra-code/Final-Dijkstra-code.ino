#include <Arduino.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include <limits.h>

#define NUM_SENSORS 4
//#define BTN_PIN A2
// Adjacency matrix representing distances between sensors
int graph[NUM_SENSORS][NUM_SENSORS] = {
  { 0, 20, 10, 25 },
  { 20, 0, 25, 10 },
  { 10, 25, 0, 20 },
  { 25, 10, 20, 0 }
};

// int graph[NUM_SENSORS][NUM_SENSORS] = {
//   { 0, 20, 10 },
//   { 20, 0, 25 },
//   { 10, 25, 0 }
// };

//Define ultrasonic sensor, servo, and pin arrays
const int triggerPins[NUM_SENSORS] = { 2, 4, 6, 8 };
const int echoPins[NUM_SENSORS] = { 3, 5, 7, 9 };
const int servoPins[NUM_SENSORS] = { 10, 11, 12, 13 };

// const int triggerPins[NUM_SENSORS] = { 2, 4, 6 };
// const int echoPins[NUM_SENSORS] = { 3, 5, 7 };
// const int servoPins[NUM_SENSORS] = { 10, 11, 12 };

Ultrasonic sensors[NUM_SENSORS] = {
  Ultrasonic(triggerPins[0], echoPins[0]),
  Ultrasonic(triggerPins[1], echoPins[1]),
  Ultrasonic(triggerPins[2], echoPins[2]),
  Ultrasonic(triggerPins[3], echoPins[3])
};

Servo servos[NUM_SENSORS];

int minDistance(int dist[], bool Tset[]) {
  int min = INT_MAX, min_index;

  for (int i = 0; i < NUM_SENSORS; i++)
    if (Tset[i] == false && dist[i] <= min)
      min = dist[i], min_index = i;


  return min_index;
}

// Function to find the shortest path using Dijkstra's algorithm
void dijkstraAlgorithm(int src) {
  int dist[NUM_SENSORS];  // creating a list to store the shortest distances for each vertex
  bool Tset[NUM_SENSORS];

  for (int i = 0; i < NUM_SENSORS; i++)
    dist[i] = INT_MAX, Tset[i] = false;

  dist[src] = 0;  // this is because the distance from source to source is zero

  // Finding shortest paths for each vertex
  for (int count = 0; count < NUM_SENSORS - 1; count++) {
    int u = minDistance(dist, Tset);
    // Mark the vertex as visited
    Tset[u] = true;
    for (int v = 0; v < NUM_SENSORS; v++)
      if (!Tset[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v])
        dist[v] = dist[u] + graph[u][v];
  }
  int orderOfServos[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; ++i) {
    orderOfServos[i] = i;
  }
  for (int i = 0; i < NUM_SENSORS - 1; ++i) {
    for (int j = i + 1; j < NUM_SENSORS; ++j) {
      if (dist[orderOfServos[i]] > dist[orderOfServos[j]]) {
        int temp = orderOfServos[i];
        orderOfServos[i] = orderOfServos[j];
        orderOfServos[j] = temp;
      }
    }
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    servos[orderOfServos[i]].write(100);  // Ensure the servo is at 0 degrees at the end
    delay(500);
    servos[orderOfServos[i]].write(0);  // Ensure the servo is at 0 degrees at the end
    delay(500);
  }
}

void setup() {
  Serial.begin(9600);
  //pinMode(BTN_PIN, OUTPUT);
  //digitalWrite(BTN_PIN, HIGH);

  // Attach servo motors to pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(0);
  }
}

void loop() {
  int detectedSensor = -1;
  float minDistance = 6;

  // if (digitalRead(BTN_PIN) == LOW) {
  //   //Serial.println("Button is pressed");
  //   for (int i = 0; i < NUM_SENSORS; i++) {
  //     servos[i].write(100);
  //   }
  // }

  for (int i = 0; i < NUM_SENSORS; i++) {
    float distance = sensors[i].read();
    if (distance < minDistance) {
      detectedSensor = i;
    }
  }
  // Serial.print('detectedSensor');
  // Serial.println(detectedSensor);

  if (detectedSensor != -1) {
    dijkstraAlgorithm(detectedSensor);
    //detectedSensor = -1;
  }
  delay(500);
}
