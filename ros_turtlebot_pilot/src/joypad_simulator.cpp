#include "joypad_simulator.hh"
#define CTRL_C 3
#define Q_ASCII 113
#define PUBLISH 10

/* prepare the map with button
 * Complex movements:
 * u	i	o
 * j	k	l
 * n	m	,
 * 
 * Simple movements with arrow
 * 
 * Right joystick:
 * q	w	e
 * a	s	d
 * z	x	c
 * 
 * 
 * 
 */
void JoyPadSimulator::Prepare()
{
    // initialization of button map with correct indexes
    buttonMap[int('s')] = 0;
    buttonMap[int('a')] = 1;
    buttonMap[int('d')] = 2;
    buttonMap[int('w')] = 3;
    buttonMap[int('x')] = 4;
    
    buttonMap[int('r')] = 0;
    buttonMap[int('t')] = 1;
    buttonMap[int('y')] = 2;
    buttonMap[int('f')] = 3;
    buttonMap[int('v')] = 4;
    
    // associate to the key map the value for linear and angular velocity
    
    // initialization of arrays for directions
    // left joystick
    std::vector<int> forward={0,1,0,0};
    std::vector<int> backward = {0,-1,0,0};
    std::vector<int> right = {0,0,0,1};
    std::vector<int> left  = {0,0,0,-1};
    std::vector<int> forward_left = {0,1,0,-1};
    std::vector<int> forward_right = {0,1,0,1};
    std::vector<int> backward_left = {0,-1,0,-1};
    std::vector<int> backward_right= {0,-1,0,1};
    std::vector<int> stay  = {0,0,0,0};
    // right joystick
     std::vector<int> forward_r ={1,0,0,0};
    std::vector<int> backward_r = {-1,0,0,0};
    std::vector<int> right_r = {0,0,1,0};
    std::vector<int> left_r  = {0,0,-1,0};
    std::vector<int> forward_left_r = {1,0,-1,0};
    std::vector<int> forward_right_r = {1,0,1,0};
    std::vector<int> backward_left_r= {-1,0,-1,0};
    std::vector<int> backward_right_r= {-1,0,1,};
    
    // left joystick association
    arrowMap[KEY_LEFT] = left;
    arrowMap[int('j')] = left;
    arrowMap[KEY_RIGHT] = right;
    arrowMap[int('l')] = right;
    arrowMap[KEY_UP] = forward;
    arrowMap[int('i')] = forward;
    arrowMap[KEY_DOWN] = backward;
    arrowMap[int('m')] = backward;
    arrowMap[int('k')] = stay;
    
    // combination of both linear and angular
    arrowMap[int('u')] = forward_left;
    arrowMap[int('o')] = forward_right;
    arrowMap[int('n')] = backward_left;
    arrowMap[int(',')] = backward_right;
    
    // right joystick association
    arrowMap[int('w')] = forward_r;
    arrowMap[int('x')] = backward_r;
    arrowMap[int('a')] = left_r;
    arrowMap[int('d')] = right_r;
    
    // combination of right joystick
    arrowMap[int('q')] = forward_left_r;
    arrowMap[int('z')] = backward_left_r;
    arrowMap[int('c')] = backward_right_r;
    arrowMap[int('e')] = forward_right_r;
    arrowMap[int('s')] = stay;
    
   // publish on the topic joy
    pub = nodeHandle.advertise<sensor_msgs::Joy>("joy", 10);
}

void JoyPadSimulator::Run()
{
    int c = 0;
    int status = 0;
    setupTerminal();
    printMenu();
    sensor_msgs::Joy msg;
    while(ros::ok() && c!=Q_ASCII && c!=CTRL_C){
      if(status == 0){
	msg = initMsg();
      }
      
      // init header
      msg.header.stamp= ros::Time().now();
      
      // get character
      c = getch();
      
      if(c!=ERR){
	// try to get an arrow from keyMap
	try{
	  std::vector<int> arrows = arrowMap.at(c);
	  msg.axes[0] = arrows[0];
	  msg.axes[1] = arrows[1];
	  msg.axes[3] = arrows[3];
	  msg.axes[4] = arrows[4];
	}
	catch (const std::out_of_range& oor){
	};
	
	// try get a button
	try{
	  msg.buttons.at(buttonMap.at(c))=1;
	}catch (const std::out_of_range& oor) {
	};
      }
      
      // for supporting multiple keys in the same msg
      status++;
      
      if(status == PUBLISH){
	// publish message
	pub.publish(msg);
	status = 0;
      }
      
    }
    
   // restore terminal normal mode
    restoreTerminal();
    
    // print ctrl c
    if(c==CTRL_C || c==Q_ASCII){
      std::cout <<" Quit \n\n";
    }
}

sensor_msgs::Joy JoyPadSimulator::initMsg()
{
    sensor_msgs::Joy msg = sensor_msgs::Joy();
    // init arrays of axes and buttons
    msg.axes.push_back(0);
    msg.axes.push_back(0);
    msg.axes.push_back(0);
    msg.axes.push_back(0);
    
    std::map<int,int>::iterator mapItr;
    // reserve a button for each button in button map
    for(mapItr=buttonMap.begin();mapItr!= buttonMap.end();mapItr++){
      msg.buttons.push_back(0);
    }
    
    return msg;
}


void JoyPadSimulator::setupTerminal(){
  // init functions
  initscr();
  // get functions and arrow
  keypad(stdscr,true);
  // do not wait enter, get also signal like exit
  raw();
  // do not print characters on terminal
  noecho();
  // setup a delay for keypress, if no key pressed the getch returns ERR
  halfdelay(1);
  
}



void JoyPadSimulator::restoreTerminal(){
  // reset terminal normal mode
   endwin();
}

void JoyPadSimulator::printMenu()
{
  printw("Welcome to JoyPad Simulator node\n");
  printw("Supported Keys:\n");
  printw("\t-\tLeft, Right, Up, Down arrows for moving\n");
  printw("\t-\t t decrease angular velocity\n");
  printw("\t-\t y increase angular velocity\n");
  printw("\t-\t f increase linear velocity\n");
  printw("\t-\t v decrease linear velocity\n");
  printw("\t-\t r switch mode\n");
  printw("For more complex movement:\n");
  printw("\t-\t u i o\n");
  printw("\t \t j k l\n");
  printw("\t \t n m ,\n");
  printw("The second joypad is mapped to the following keys:\n");
  printw("\t-\t q w e\n");
  printw("\t \t a s d\n");
  printw("\t \t z x c\n");
  
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "joypad_simulator");
  JoyPadSimulator mNode;
   
  mNode.Prepare();
  mNode.Run();
  
  return (0);
}


