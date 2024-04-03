#ifndef CROSSCORR
#define CROSSCORR
    //c standard library
    #include <cstdlib>
    #include <iostream>
    #include <complex>
    #include <string>
    #include <vector>
    #include <tuple>
    #include <cmath>
    #include <typeinfo>

    //using buffer namespace
    #include "../BufferHandler.hpp"

    using Buffers::Buffer_1D;

    namespace CrossCorr_namespace{

        template<typename datatype>
        class CrossCorr
        {
        public:
            //buffer to store the result of the cross corrrelation
            Buffer_1D<std::complex<datatype>> result;

            //parameters to use when computing the cross correlation
            Buffer_1D<int> lags;

            //paramet to track the number of delay samples
            int delay_samples;
        
        private:
            size_t max_lag;
            bool normalize;

            //flag to track if the cross correlation has been initialized
            bool initialized;
            
        public:

            /**
             * @brief Construct a new Cross Corr object
             * 
             */
            CrossCorr(bool enable_normalization = true):
                result(),
                lags(),
                delay_samples(0),
                max_lag(0),
                normalize(enable_normalization),
                initialized(false)
                {}


            /**
             * @brief Construct a new Cross Corr object
             * 
             * @param desired_max_lag Desired maximum lag to evaluate
             */
            CrossCorr(size_t desired_max_lag,bool enable_normalization = true):
                result((2 * desired_max_lag) + 1),
                lags((2 * desired_max_lag) + 1),
                delay_samples(0),
                max_lag(desired_max_lag),
                normalize(enable_normalization),
                initialized(true)
            {
                compute_lag_values();
            }

            ~CrossCorr() {};
            
            /**
             * @brief Set the max lag value
             * 
             * @param desired_max_lag desired maximum lag value
             */
            void set_max_lag(size_t desired_max_lag){
                max_lag = desired_max_lag;
                lags.set_buffer_size((2 * desired_max_lag) + 1);
                result.set_buffer_size((2 * max_lag) + 1);
                compute_lag_values();
                initialized = true;
            }

            /**
             * @brief Compute the cross correlation of two input vectors (assumes the vectors are of the same size)
             * 
             * @param x complex vector containing samples for x
             * @param y complex vector containing samples for y
             */
            void compute(
                std::vector<std::complex<datatype>> & x,
                std::vector<std::complex<datatype>> & y)
            {
                //check to make sure that max_lag value is valid and initialized
                if(!(initialized) || (max_lag > (2 * x.size()) - 1)){
                    max_lag = x.size();
                    lags.set_buffer_size(max_lag);
                    result.set_buffer_size((2 * max_lag) + 1);
                    compute_lag_values();
                    initialized = true;
                }

                //compute cross correlation at each value of m
                int m = 0;
                for (size_t i = 0; i < lags.num_samples; i++)
                {
                    m = lags.buffer[i];
                    if (m >= 0)
                    {
                        result.buffer[i] = R_xy(m,x,y);
                    }
                    else{
                        result.buffer[i] = std::conj(R_xy(-1 * m,y,x));
                    }
                }

                if (normalize)
                {
                    normalize_result(x,y);
                }

                compute_delay_samples();
            }

            /**
             * @brief Conver the delay in samples to a delay in us using the given sample rate
             * 
             * @param sample_rate_MSps sample rate in MSps
             * @return datatype the delay in us
             */
            double compute_delay_us(double sample_rate_MSps){
                return static_cast<double>(delay_samples) / sample_rate_MSps;
            }

        private:

            /**
             * @brief Function to compute the lag values used when computing the cross correlation
             *  (assumes that max_lag and lags buffer has already been initialized)
             * 
             */
            void compute_lag_values(){
                
                int lag = static_cast<int>(max_lag) * -1;

                for (size_t i = 0; i < lags.num_samples; i++)
                {
                    lags.buffer[i] = lag;
                    lag ++;
                }
            }

            /**
             * @brief Compute the cross correlation at a single index m
             * (assumes x and y are the same length)
             * 
             * @param m index to compute the cross correlation at (assumed to be positive)
             * @param x complex vector containing samples for x
             * @param y complex vector containing samples for y
             * @return std::complex<datatype> result of computation
             */
            std::complex<datatype> R_xy(
                int m,
                std::vector<std::complex<datatype>> & x,
                std::vector<std::complex<datatype>> & y)
            {
                //determine the size of x and y
                int N = x.size();

                //initialize result variable
                std::complex<datatype> sum(0,0);

                for (int i = 0; i < (N - m - 1); i++)
                {
                    if ((i + m) < N)
                    {
                        sum += x[i + m] * std::conj(y[i]);
                    }
                    else{
                        sum += x[i + m - N] * std::conj(y[i]);
                    }
                }

                //return the final sum
                return sum;
            }

            /**
             * @brief Compute the complex normalization coefficient
             * 
             * @param x complex vector constaining samples for x
             * @param y complex vector containing samples for y
             * @return std::complex<datatype> 
             */
            std::complex<datatype> compute_normalization_coeff(
                std::vector<std::complex<datatype>> & x,
                std::vector<std::complex<datatype>> & y)
            {
                datatype R_xx = std::abs(R_xy(0,x,x));
                datatype R_yy = std::abs(R_xy(0,y,y));

                //compute normalization coefficient
                datatype normalization_coeff = 1 / (std::sqrt(R_xx * R_yy));

                //return the normalization coefficient as a complex number
                return std::complex<datatype>(normalization_coeff,0);
            }

            /**
             * @brief Normalizes the result of the cross-correlation
             * 
             * @param x complex vector containing samples for x
             * @param y complex vector containing samples for y
             */
            void normalize_result(
                std::vector<std::complex<datatype>> & x,
                std::vector<std::complex<datatype>> & y)
            {
                //compute the normalization coefficient
                std::complex<datatype> normalization_coefficient = compute_normalization_coeff(x,y);

                for (size_t i = 0; i < result.num_samples; i++)
                {
                    result.buffer[i] = result.buffer[i] * normalization_coefficient;
                }
            }

            /**
             * @brief compute the delay in samples after the cross correlation has been performed
             * 
             */
            void compute_delay_samples(){

                datatype max_val = 0;
                delay_samples = 0;

                //go through the data and look for the larges value in the cross correlation value by absolute value
                for (size_t i = 0; i < result.num_samples; i++)
                {
                    if (std::abs(result.buffer[i]) > max_val)
                    {
                        max_val = std::abs(result.buffer[i]);
                        delay_samples = lags.buffer[i];
                    }   
                }
            }

            

        };
    }

#endif