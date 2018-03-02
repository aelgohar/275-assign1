#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <SD.h>
#include "consts_and_types.h"
#include "map_drawing.h"

// the variables to be shared across the project, they are declared here!
shared_vars shared;

Adafruit_ILI9341 tft = Adafruit_ILI9341(clientpins::tft_cs, clientpins::tft_dc);

void setup() {
  // initialize Arduino
  init();

  // initialize zoom pins
  pinMode(clientpins::zoom_in_pin, INPUT_PULLUP);
  pinMode(clientpins::zoom_out_pin, INPUT_PULLUP);

  // initialize joystick pins and calibrate centre reading
  pinMode(clientpins::joy_button_pin, INPUT_PULLUP);
  // x and y are reverse because of how our joystick is oriented
  shared.joy_centre = xy_pos(analogRead(clientpins::joy_y_pin), analogRead(clientpins::joy_x_pin));

  // initialize serial port
  Serial.begin(9600);
  Serial.flush(); // get rid of any leftover bits

  // initially no path is stored
  shared.num_waypoints = 0;

  // initialize display
  shared.tft = &tft;
  shared.tft->begin();
  shared.tft->setRotation(3);
  shared.tft->fillScreen(ILI9341_BLUE); // so we know the map redraws properly

  // initialize SD card
  if (!SD.begin(clientpins::sd_cs)) {
      Serial.println("Initialization has failed. Things to check:");
      Serial.println("* Is a card inserted properly?");
      Serial.println("* Is your wiring correct?");
      Serial.println("* Is the chipSelect pin the one for your shield or module?");

      while (1) {} // nothing to do here, fix the card issue and retry
  }

  // initialize the shared variables, from map_drawing.h
  // doesn't actually draw anything, just initializes values
  initialize_display_values();

  // initial draw of the map, from map_drawing.h
  draw_map();
  draw_cursor();

  // initial status message
  status_message("FROM?");
}

void process_input() {
  // read the zoom in and out buttons
  shared.zoom_in_pushed = (digitalRead(clientpins::zoom_in_pin) == LOW);
  shared.zoom_out_pushed = (digitalRead(clientpins::zoom_out_pin) == LOW);

  // read the joystick button
  shared.joy_button_pushed = (digitalRead(clientpins::joy_button_pin) == LOW);

  // joystick speed, higher is faster
  const int16_t step = 64;

  // get the joystick movement, dividing by step discretizes it
  // currently a far joystick push will move the cursor about 5 pixels
  xy_pos delta(
    // the funny x/y swap is because of our joystick orientation
    (analogRead(clientpins::joy_y_pin)-shared.joy_centre.x)/step,
    (analogRead(clientpins::joy_x_pin)-shared.joy_centre.y)/step
  );
  delta.x = -delta.x; // horizontal axis is reversed in our orientation

  // check if there was enough movement to move the cursor
  if (delta.x != 0 || delta.y != 0) {
    // if we are here, there was noticeable movement

    // the next three functions are in map_drawing.h
    erase_cursor();       // erase the current cursor
    move_cursor(delta);   // move the cursor, and the map view if the edge was nudged
    if (shared.redraw_map == 0) {
      // it looks funny if we redraw the cursor before the map scrolls
      draw_cursor();      // draw the new cursor position
    }
  }
}

bool read_from_serial(char arr[], long long delayTime){
  int used = 0; // iterates through the buffer array
  long long curtime = millis();  // checks delay
  int returnVal = 1;
  Serial.flush();
  while((millis() - curtime) <= delayTime){
    while(Serial.available()==0){}  // waits

    // Serial.print("reached reading lines");
    char c = Serial.read(); // read a byte
    arr[used++] = c;  // add it to array
    // Serial.print(arr);
    if(c == '\n'){
      arr[used]=='\0';  // null terminate it
      returnVal = 0;// did NOT time out

      break;
    }
  }
  return returnVal;
}

void interact_with_server(lon_lat_32 start, lon_lat_32 end){
  enum {SENDING_REQUEST, WAITING_FOR_NUMBER, RECEIVING_WAYPOINTS, DONE} curr_mode_comm = SENDING_REQUEST;
  lon_lat_32 tempStart = start;
  lon_lat_32 tempEnd = end;
  int iteration;

  while(true){
    if (curr_mode_comm == SENDING_REQUEST){
      Serial.print("R ");
      Serial.print(tempStart.lat);
      Serial.print(" ");
      Serial.print(tempStart.lon);
      Serial.print(" ");
      Serial.print(tempEnd.lat);
      Serial.print(" ");
      Serial.println(tempEnd.lon);
      curr_mode_comm = WAITING_FOR_NUMBER;
    }

    else if (curr_mode_comm == WAITING_FOR_NUMBER){
      char buffer[129];
      bool timed_out = read_from_serial(buffer, 10000);
      if (!timed_out && buffer[0] == 'N'){
        status_message("recieving number");
        shared.num_waypoints = buffer[2];
        iteration = shared.num_waypoints;
        if(shared.num_waypoints >= max_waypoints){
          curr_mode_comm = DONE;
        }
        Serial.println('A');
        curr_mode_comm = RECEIVING_WAYPOINTS;
      }
      else if (timed_out){
        curr_mode_comm = SENDING_REQUEST;
      }
    }

    else if (curr_mode_comm == RECEIVING_WAYPOINTS){
      char buffer[129];
      // only 1 second delays
      bool timed_out = read_from_serial(buffer, 2000);
      status_message("recieving waypoints");

      if (!timed_out && buffer[0] == 'W'){
        Serial.println('A');
        char* endp;
        int32_t temp_lon = strtol(&buffer[2], &endp, 10);
        int32_t temp_lat = strtol(endp, NULL, 10);

        shared.waypoints[shared.num_waypoints - iteration].lon = temp_lon;
        shared.waypoints[shared.num_waypoints - iteration--].lat = temp_lat;
        status_message("waypoints");
      }

      else if(buffer[0] == 'E'){
        curr_mode_comm = DONE;
      }

      else if (timed_out){
        status_message("timed out1");
        curr_mode_comm = SENDING_REQUEST;
      }
    }
    else if (curr_mode_comm == DONE){
      break;
    }
  }
}
void drawPath(lon_lat_32 start, lon_lat_32 end){
  int32_t x1 = constrain(longitude_to_x(shared.map_number, start.lon) - shared.map_coords.x , 0 , displayconsts::display_width);
  int32_t x2 = constrain(longitude_to_x(shared.map_number, end.lon) - shared.map_coords.x , 0 , displayconsts::display_width);
  int32_t y1 = constrain(latitude_to_y(shared.map_number, start.lon) - shared.map_coords.y , 0 , displayconsts::display_height);
  int32_t y2 = constrain(latitude_to_y(shared.map_number, end.lon) - shared.map_coords.y , 0 , displayconsts::display_height);
  shared.tft->drawLine(x1, y1, x2, y2, ILI9341_BLUE);
}
int main() {
  setup();

  // very simple finite state machine:
  // which endpoint are we waiting for?
  enum {WAIT_FOR_START, WAIT_FOR_STOP} curr_mode = WAIT_FOR_START;

  // the two points that are clicked
  lon_lat_32 start, end;

  while (true) {
    // clear entries for new state
    shared.zoom_in_pushed = 0;
    shared.zoom_out_pushed = 0;
    shared.joy_button_pushed = 0;
    shared.redraw_map = 0;

    // reads the three buttons and joystick movement
    // updates the cursor view, map display, and sets the
    // shared.redraw_map flag to 1 if we have to redraw the whole map
    // NOTE: this only updates the internal values representing
    // the cursor and map view, the redrawing occurs at the end of this loop
    process_input();

    // if a zoom button was pushed, update the map and cursor view values
    // for that button push (still need to redraw at the end of this loop)
    // function zoom_map() is from map_drawing.h
    if (shared.zoom_in_pushed) {
      zoom_map(1);
      shared.redraw_map = 1;
    }
    else if (shared.zoom_out_pushed) {
      zoom_map(-1);
      shared.redraw_map = 1;
    }

    // if the joystick button was clicked
    if (shared.joy_button_pushed) {

      if (curr_mode == WAIT_FOR_START) {
        // if we were waiting for the start point, record it
        // and indicate we are waiting for the end point
        start = get_cursor_lonlat();
        curr_mode = WAIT_FOR_STOP;
        status_message("TO?");

        // wait until the joystick button is no longer pushed
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
      }
      else {
        // if we were waiting for the end point, record it
        // and then communicate with the server to get the path
        while (digitalRead(clientpins::joy_button_pin) == HIGH) {}
        end = get_cursor_lonlat();
        interact_with_server(start, end);

        status_message("here");
        for (int z = 0; z < 1; z++){
          // shared.tft->drawLine((longitude_to_x(shared.map_number, shared.waypoints[z].lon)), (latitude_to_y(shared.map_number, shared.waypoints[z].lat)),
          // (longitude_to_x(shared.map_number, shared.waypoints[z+1].lon)), (latitude_to_y(shared.map_number, shared.waypoints[z+1].lat)), ILI9341_BLUE);
          drawPath(shared.waypoints[z], shared.waypoints[z+1]);
        }


        curr_mode = WAIT_FOR_START;

        // wait until the joystick button is no longer pushed
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
      }
    }

    if (shared.redraw_map) {
      // redraw the status message
      if (curr_mode == WAIT_FOR_START) {
        status_message("FROM?");
      }
      else {
        status_message("TO?");
      }

      // redraw the map and cursor
      draw_map();
      draw_cursor();

      // TODO: draw the route if there is one
    }
  }

  Serial.flush();
  return 0;
}
