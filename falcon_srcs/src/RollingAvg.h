/*
 * RollingAvg.h
 *
 * Created: 2/16/2024 5:15:43 PM
 *  Author: Daniel
 *  Project: Falcon Alert
 */ 


#ifndef ROLLINGAVG_H_
#define ROLLINGAVG_H_

template <typename T>
class RollingAvg
{
public:
  
  // class constructor, requires the desired number of samples to hold in the array.
  // dynamically sizes the average value array and default inits all values to suppled init_value.
  RollingAvg(uint8_t num_samples, T init_val=0) : m_size(num_samples)
  {    
    m_avg_array = new T [m_size];
    avg_value = init_val;
    fill(init_val);
  }
  
  // fill the average array with a single value, useful during initialization
  void fill(T value)
  {
    for(uint8_t i=0; i<m_size; i++)
    {
      m_avg_array[i] = value;
    }
    calc_avg();
  }
  
  // get the size of the rolling average, if you need it for some reason
  uint8_t size()
  {
    return m_size;
  }
  
  T avg()
  {
    return avg_value;
  }

  T get(uint8_t index) 
  {
    return m_avg_array[(spot+index)%m_size];
  }
  
  // add a new value to the rolling average array
  void add(T new_val)
  {
    m_avg_array[spot%m_size] = new_val;
    spot++;
    calc_avg();
  }
  
  // add a new value and return the new rolling average
  T addAvg(T new_val)
  {
    add(new_val);
    calc_avg();
    return avg();
  }
  
  // destructor, frees the dynamically allocated memory.
  ~RollingAvg()
  {
    delete[] m_avg_array;
  }
  
private:

  const uint8_t m_size;
  T *m_avg_array;
  uint8_t spot;
  T avg_value;

  // get the current rolling average
  void calc_avg()
  {
    T sum = (T)0;
    for(int i=0; i<m_size; i++)
    {
      sum += m_avg_array[i];
    }
    
    avg_value = ((T)sum/(T)m_size);
  }
};



#endif /* ROLLINGAVG_H_ */