# ifndef TIME_LOGGER_H
# define TIME_LOGGER_H

# include <stdio.h>
# include <vector>
# include <chrono>
# include <string>

# define TimeType std::chrono::system_clock::time_point
# define NOW() std::chrono::high_resolution_clock::now()

double Diff_ms(TimeType start, TimeType finish);



struct MinMax
{
    double min;
    double max;
    std::string name;
    int count;
    double accumulated;    
    int skip;
    
    MinMax();

    MinMax(std::string id);
    
    void Print(const char* name);
    
    void Update(double value);
    
};



struct TimeLogger
{
    struct EventPoint
    {
        int line_number;
        std::string event_name;
        TimeType time;
    };
    TimeType start;
    std::vector<EventPoint> time_points;

	void Start();

	void Log(int line, std::string name);

	void Log(std::string name);

	double Display();
};

extern TimeLogger g_time_logger;

# if 1

# define START_TIMER() g_time_logger.Start()

# define LOG(x) g_time_logger.Log(__LINE__, #x)

# define DISPLAY_LOG() g_time_logger.Display();

# else

# define START_TIMER() ;

# define LOG(x) ;

# define DISPLAY_LOG() ;

# endif

# endif