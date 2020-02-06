#ifndef PWMCONTROLfourS_H_
#define PWMCONTROLfourS_H_

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <vector>
#include <unistd.h>
#include <fstream>
#include <string>
#include <sstream>

// Prop 8045 dji drones on 3S battery
#define minpwm 1000000
#define maxpwm 1800000


#define MAX_BUF 64

bool write2file(const std::string& filepath, const std::string& datastr ){
	fd = open(filepath.c_str(), O_WRONLY);
    write(fd, datastr.c_str(), datastr.size()*sizeof(char));
    close(fd);
}

std::string readfile(const std::string& filepath){
	std::string datastr;
	fd = open(filepath.c_str(), O_RONLY);
    read(fd, datastr.c_str(), datastr.size()*sizeof(char));
    close(fd);

    return datastr;
}

std::string getPwmExportStr(unsigned short int Chipno){
	return "/sys/class/pwm/pwmchip"+std::to_string(Chipno)+"/export";
}
std::string getPwmPolarityStr(unsigned short int Chipno,unsigned short int pwmno){
	return "/sys/class/pwm/pwm"+std::to_string(Chipno)+":"+std::to_string(pwmno)+ "/polarity";
}
std::string getPwmPeriodStr(unsigned short int Chipno,unsigned short int pwmno){
	return "/sys/class/pwm/pwm"+std::to_string(Chipno)+":"+std::to_string(pwmno)+ "/period";
}
std::string getPwmEnableStr(unsigned short int Chipno,unsigned short int pwmno){
	return "/sys/class/pwm/pwm"+std::to_string(Chipno)+":"+std::to_string(pwmno)+ "/enable";
}
std::string getPwmDutyStr(unsigned short int Chipno,unsigned short int pwmno){
	return "/sys/class/pwm/pwm"+std::to_string(Chipno)+":"+std::to_string(pwmno)+ "/duty_cycle";
}

std::string getGpioExportStr(){
	return "/sys/class/gpio/export";
}
std::string getGpioEnableStr(unsigned short int gpiono){
	return "/sys/class/gpio/gpio"+std::to_string(gpiono)+"/enable";
}
std::string getGpioDirnStr(unsigned short int gpiono){
	return "/sys/class/gpio/gpio"+std::to_string(gpiono)+"/direction";
}


class GPIOgroup{
	std::vector<unsigned short int> _GPIOpins;
public:
	GPIOgroup(std::vector<unsigned short int> GPIOpins):_GPIOpins(GPIOpins){

	}
	void export_gpios(){
		for(auto gpiono& : _GPIOpins ){
			write2file(getGpioExportStr(), std::to_string(gpiono) );
		}
	}
	void enable_pins(){
		for(auto gpiono& _GPIOpins ){
			write2file(getGpioEnableStr(gpiono), std::to_string(1) );
		}

	}
	set_pins(std::vector<unsigned short int> pinindicies){
		for(auto pind& : pinindicies ){
			auto gpiono = _GPIOpins[pind];
			write2file(getGpioDirnStr(gpiono), std::to_string(1) );
		}
	}
	set_all(){
		for(auto gpiono& _GPIOpins ){
			write2file(getGpioDirnStr(gpiono), std::to_string(1) );
		}

	}
	clear_pins(std::vector<unsigned short int> pinindicies){
		for(auto pind& : pinindicies ){
			auto gpiono = _GPIOpins[pind];
			write2file(getGpioDirnStr(gpiono), std::to_string(0) );
		}
	}
	clear_all(){
		for(auto gpiono& _GPIOpins ){
			write2file(getGpioDirnStr(gpiono), std::to_string(0) );
		}

	}
	toggle_pins(std::vector<unsigned short int> pinindicies){
		for(auto pind& : pinindicies ){
			auto gpiono = _GPIOpins[pind];
			auto ss = readfile(getGpioDirnStr(gpiono));
			if(ss=="1")
				write2file(getGpioDirnStr(gpiono), std::to_string(0) );
			else
				write2file(getGpioDirnStr(gpiono), std::to_string(1) );

		}
	}
};


enum class THREADDIRN{
FWD 1,
BKWD -1
};

class PWMgroup{
	 unsigned long int _period;
	 unsigned long int maxApplyMilliSec = 5000;
	 unsigned short int _Chipno;
	 std::vector<unsigned short int> _Pwmno;
	 GPIOgroup _GPIOgroup;
	 unsigned long int _period;
	 GpioCtrlPins _GpioCtrlPins;

	 int Npwms=0;


public:
	PWMgroup(unsigned short int Chipno, 
				std::vector<unsigned short int> Pwmnos, 
				std::vector<unsigned short int> GpioCtrlPins, 
				unsigned long int period):_GPIOgroup(GpioCtrlPins){

		_Chipno = Chipno;
		_Pwmnos = Pwmnos;
		_period = period;
		

		Npwms = Pwmnos.size();
		_GPIOgroup.export_gpios();
		export_pwms();
	}
	

	void export_pwms(){
		int fd, len;
		char buf[MAX_BUF];
	    // struct stat st;

		for(cont auto& pwmno :_Pwmnos){
			write2file(getPwmExportStr(_Chipno), std::to_string(pwmno) );
			write2file(getPwmPolarityStr(_Chipno,pwmno), std::to_string(0) );
			write2file(getPwmPeriodStr(_Chipno,pwmno), std::to_string(_period) );
			write2file(getPwmDutyStr(_Chipno,pwmno), std::to_string(0) );
			write2file(getPwmEnableStr(_Chipno,pwmno), std::to_string(1) );
		    usleep(1000);
		}


		}


		
	}

void set_duties_percent(std::vector<unsigned long int> duties_percent, ,unsigned long int applyForMilliSeconds){
	std::vector<unsigned long int> duties;
	std::vector<THREADDIRN> dirns(2,THREADDIRN::FWD);
	for(int i=0;i<duties_percent.size();++i){
		if(duties_percent[i]>=0)
			dirns[i] = THREADDIRN::FWD;
		else
			dirns[i] = THREADDIRN::BKWD;

		auto x = std:abs(duties_percent[i])*_period;
		x = max(minpwm,min(x,maxpwm));
		duties.push_back(x);
	}

	set_duties(duties,dirns,applyForMilliSeconds);
}
  void set_duties(std::vector<unsigned long int> duties, std::vector<THREADDIRN> dirns , unsigned long int applyForMilliSeconds){
	applyForMilliSeconds= max(maxApplyMilliSec,applyForMilliSeconds);

	
	for(int i=0;i<Npwms;++i){
		if( dirns[i]==THREADDIRN::FWD ){
			_GPIOgroup.clear_pin(i);
			write2file(getPwmDutyStr(_Chipno,_Pwmnos[i]), std::to_string(duties[i]) );
		} 
		else{
			_GPIOgroup.set_pin(i);
			write2file(getPwmDutyStr(_Chipno,_Pwmnos[i]), std::to_string(maxpwm - duties[i]) );
		}
	}

	usleep(applyForMilliSeconds*1e6);

	_GPIOgroup.clear_all();

	for(int i=0;i<Npwms;++i){
		write2file(getPwmDutyStr(_Chipno,_Pwmnos[i]), std::to_string(0) );
	    // usleep(1000);
	}

 }

 


};
#endif
