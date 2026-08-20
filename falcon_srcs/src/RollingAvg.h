/*
 * RollingAvg.h
 *
 * Created: 2/16/2024 5:15:43 PM
 *  Author: Daniel
 *  Project: Falcon Alert
 */


#ifndef ROLLINGAVG_H_
#define ROLLINGAVG_H_

/*
 * ⚠️ THE DEPTH IS A TEMPLATE PARAMETER, NOT A CONSTRUCTOR ARGUMENT (2026-08-20).
 *
 * This was `RollingAvg(uint8_t num_samples)` doing `m_avg_array = new T[m_size]`
 * in the constructor. Both instances are file-scope globals with compile-time
 * constant depths (32 and 8), so the allocation happened once at static-init and
 * was never freed -- but linking `new` pulled malloc AND free into the image for
 * 586 bytes, on a part that was down to 62 bytes free. It was also the last
 * dynamic allocation on the device, and the reason the vendored Wire driver
 * avoids `new` is that same 586 bytes (see lib/Wire).
 *
 * The array is now a fixed member. Same arithmetic, same API, same numerics --
 * only the storage moved from the heap to BSS, which is where it effectively
 * lived anyway.
 *
 * Fixed in passing: `spot` was never initialised by the constructor. That was
 * harmless ONLY because both instances are globals and so zero-initialised
 * before their constructors run; a stack or member instance would have started
 * at an arbitrary offset. It is initialised explicitly now, so the class no
 * longer depends on where it is declared.
 */
template <typename T, uint8_t N>
class RollingAvg
{
public:

  // class constructor. Depth is the template parameter N; init_val fills every
  // slot, so the average is well-defined from the first call.
  RollingAvg(T init_val=0)
  {
    avg_value = init_val;
    spot = 0;
    fill(init_val);
  }

  // fill the average array with a single value, useful during initialization
  void fill(T value)
  {
    for(uint8_t i=0; i<N; i++)
    {
      m_avg_array[i] = value;
    }
    calc_avg();
  }

  // get the size of the rolling average, if you need it for some reason
  uint8_t size()
  {
    return N;
  }

  T avg()
  {
    return avg_value;
  }

  T get(uint8_t index)
  {
    return m_avg_array[(spot+index)%N];
  }

  // add a new value to the rolling average array
  void add(T new_val)
  {
    m_avg_array[spot%N] = new_val;
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

private:

  T m_avg_array[N];
  uint8_t spot;
  T avg_value;

  // get the current rolling average
  void calc_avg()
  {
    T sum = (T)0;
    for(int i=0; i<N; i++)
    {
      sum += m_avg_array[i];
    }

    avg_value = ((T)sum/(T)N);
  }
};



#endif /* ROLLINGAVG_H_ */
