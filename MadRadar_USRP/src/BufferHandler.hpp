#ifndef BUFFERHANDLER
#define BUFFERHANDLER
    //c standard library
    #include <cstdlib>
    #include <iostream>
    #include <fstream>
    #include <complex>
    #include <string>
    #include <vector>
    #include <tuple>
    #include <memory>
    #include <cmath>
    #include <typeinfo>
    #include <algorithm>

/*
    //includes for JSON editing
    #include <nlohmann/json.hpp>

    using json = nlohmann::json;
*/

    namespace Buffers{

        /**
         * @brief Buffer Class - a parent class or a buffer
         * 
         * @tparam data_type - type of data that the buffer stores
         */
        template<typename data_type>
        class Buffer {

            //variables
            //read and write file paths
            public:
                std::string read_file;
                std::string write_file;

            //file streams
                std::shared_ptr<std::ifstream> read_file_stream;
                std::shared_ptr<std::ofstream> write_file_stream;

            //status
                bool buffer_init_status;
                bool debug_status;

            //functions
            public:
                /**
                 * @brief Default constructor a new Buffer object
                 * 
                 */
                Buffer(): debug_status(false),buffer_init_status(false) {}

                /**
                 * @brief Construct a new Buffer object, alternative constructor to specify debug status
                 * 
                 * @param debug the desired debug status
                 * @param debug the initialization status of the buffer
                 */
                Buffer(bool init, bool debug) 
                    : debug_status(debug),buffer_init_status(init) {}

                /**
                 * @brief Destroy the Buffer object
                 * 
                 */
                virtual ~Buffer(){
                    //only close the read file stream if it is the 
                    //final pointer pointing to the streaming object
                    if (read_file_stream.use_count() == 1)
                    {
                        if (read_file_stream -> is_open()){
                            read_file_stream -> close();
                        }
                    }
                    
                    
                    if (write_file_stream.use_count() == 1){
                        if (write_file_stream -> is_open()){
                            write_file_stream -> close();
                        }
                    }
                }

                /**
                 * @brief Copy Constructor
                 * 
                 * @param rhs reference to an existing Buffer Object
                 */
                Buffer(const Buffer<data_type> & rhs) : read_file(rhs.read_file),
                                                        write_file(rhs.write_file),
                                                        read_file_stream(rhs.read_file_stream),
                                                        write_file_stream(rhs.write_file_stream),
                                                        debug_status(rhs.debug_status),
                                                        buffer_init_status(rhs.buffer_init_status)
                                                        {}

                /**
                 * @brief Assignment operator support
                 * 
                 * @param rhs 
                 * @return Buffer& 
                 */
                Buffer & operator=(const Buffer<data_type> & rhs){
                    if (this != &rhs){
                        //update the read and write file names
                        read_file = rhs.read_file;
                        write_file = rhs.write_file;

                        //update the debug and buffer init status variables
                        debug_status = rhs.debug_status;
                        buffer_init_status = rhs.buffer_init_status;

                        //close the existing read/write file streams if needed
                        if (read_file_stream.use_count() == 1)
                        {
                            if (read_file_stream -> is_open()){
                                read_file_stream -> close();
                            }
                        }
                        
                        
                        if (write_file_stream.use_count() == 1){
                            if (write_file_stream -> is_open()){
                                write_file_stream -> close();
                            }
                        }

                        //set the new read and write file streams
                        read_file_stream = rhs.read_file_stream;
                        write_file_stream = rhs.write_file_stream;
                    }

                    return *this;
                }

                //managing and initializing file streams

                /**
                 * @brief set the read file path
                 * 
                 * @param file_path the file name to be read
                 * @param init_stream (on true) also initializes the read file stream,
                 * (on false) only sets the read file name
                 */
                void set_read_file(std::string file_path, bool init_stream = true){
                    read_file = file_path;
                    if (init_stream){
                        init_read_file_stream();
                    }
                }

                /**
                 * @brief Set the write file path
                 * 
                 * @param file_path the file name to be written to
                 * @param init_stream (on true) also initializes the write file stream,
                 * (on false) only sets the write file
                 */
                void set_write_file(std::string file_path, bool init_stream = true){
                    write_file = file_path;
                    if (init_stream){
                        init_write_file_stream();
                    }
                }
                
                /**
                 * @brief open the read file in preparation for streaming
                 * 
                 */
                void init_read_file_stream(void){
                    
                    if(read_file.empty()){
                        std::cerr << "Buffer::init_read_file_stream: no read_file name" <<std::endl;
                    }
                    else{
                        read_file_stream = std::make_shared<std::ifstream>(std::ifstream());
                        read_file_stream -> open(read_file.c_str(), std::ios::in | std::ios::binary);
                        if(read_file_stream -> is_open()){
                            if (debug_status)
                            {
                                std::cout << "Buffer::init_read_file_stream: read file opened successfully" <<std::endl;
                            }
                            
                        }
                        else{
                            std::cerr << "BufferHandler::failed to open read file\n";
                        }
                    }
                }

                /**
                 * @brief open the write file in preparation for streaming
                 * 
                 */
                void init_write_file_stream(void){
                    //open the write file
                    if(write_file.empty()){
                        std::cerr << "Buffer::init_write_file_stream: no write_file name" <<std::endl;
                    }
                    else{
                        write_file_stream = std::make_shared<std::ofstream>(std::ofstream());
                        write_file_stream -> open(write_file.c_str(), std::ios::out | std::ios::binary);
                        if(write_file_stream -> is_open()){
                            if(debug_status)
                            {
                                std::cout << "Buffer::init_write_file_stream: write file opened successfully" <<std::endl;
                            }
                        }
                        else{
                            std::cerr << "BufferHandler::failed to open write file\n";
                        }
                    }
                }

                /**
                 * @brief close the read file stream
                 * 
                 */
                void close_read_file_stream(void){
                    if(read_file_stream -> is_open()){
                        read_file_stream -> close();
                        if(debug_status){
                            std::cout << "Buffer::close_read_file_stream: read file stream closed" << std::endl;
                        }
                    }
                    else{
                        std::cerr << "Buffer::close_read_file_stream: close_read_file_stream"  << 
                            "called but read file already closed" << std::endl;
                    }
                }

                /**
                 * @brief close the write file stream
                 * 
                 */
                void close_write_file_stream(void){
                    if(write_file_stream -> is_open()){
                        write_file_stream -> close();
                        if(debug_status){
                            std::cout << "Buffer::close_write_file_stream: write file stream closed" << std::endl;
                        }
                    }
                    else{
                        std::cerr << "Buffer::close_write_file_stream: close_write_file_stream"  << 
                            "called but write file already closed" << std::endl;
                    }
                }

                /**
                 * @brief Read the data stored in the read_file_stream (must already be opened), 
                 * and return a vector with the data. 
                 * NOTE: The read file stream is closed after data has been loaded
                 * 
                 * @return std::vector<data_type>: a vector with the data stored in the read file
                 */
                std::vector<data_type> load_data_from_read_file(void){
                    //initialize the return vector
                    std::vector<data_type> data_vector;
                    
                    //if the read file is open, read it into the data_vector
                    if (read_file_stream -> is_open()){

                        //get the size of the file to be read
                        std::streampos size;
                        read_file_stream -> seekg (0,std::ios::end);
                        size = read_file_stream -> tellg();

                        //determine the number of samples in the file
                        size_t detected_samples = size / sizeof(data_type);
                        if(debug_status){
                            std::cout << "Buffer::load_data_from_read_file: detected samples: " << detected_samples << std::endl;
                        }
                        //define the vector
                        data_vector = std::vector<data_type>(detected_samples);

                        //read the file
                        read_file_stream -> seekg(0,std::ios::beg);
                        read_file_stream -> read((char*) &data_vector.front(), data_vector.size() * sizeof(data_type));
                        read_file_stream -> close();  
                    }
                    else{
                        std::cerr << "Buffer::load_data_from_read_file: read_file_stream is not open\n";
                    }
                    return data_vector;
                }

            //abstract functions
                virtual void print_preview(void) = 0;
                virtual void import_from_file(void) = 0;
                virtual void save_to_file() = 0;
        }; // end of Buffer class

        /**
         * @brief Buffer_2D - a 2D buffer that stores data in a buffer and allows easy read/write integration witha file
         * 
         * @tparam data_type: the type of data that the buffer stores
         */
        template<typename data_type>
        class Buffer_2D : public Buffer<data_type>{
            public:
            //variables
                //a vector to store things in
                std::vector<std::vector<data_type>> buffer;

                //keep track of the size of the buffer
                size_t num_rows;
                size_t num_cols;

                //keep track of how many excess samples are unused in the buffer
                size_t excess_samples; 
            //funcions

                /**
                 * @brief Construct a new Buffer_2D object
                 * 
                 */
                Buffer_2D(): Buffer<data_type>(){}

                /**
                 * @brief Construct a new Buffer_2D object
                 * 
                 * @param debug the specified debug setting
                 */
                Buffer_2D(bool debug)
                : Buffer<data_type>(false,debug)
                {}

                /**
                 * @brief Construct a new Buffer_2D object
                 * 
                 * @param rows the number of rows to be in the buffer
                 * @param cols the number of columns to be in the buffer
                 * @param excess the number of excess/unused samples in the buffer
                 * @param debug the specified debug setting
                 */
                Buffer_2D(size_t rows, size_t cols,size_t excess = 0, bool debug = false)
                        : Buffer<data_type>(true,debug),
                        buffer(rows,std::vector<data_type>(cols)),
                        num_rows(rows),
                        num_cols(cols),
                        excess_samples(excess){}

                
                /**
                 * @brief Construct a new Buffer_2D object- COPY Constructor
                 * 
                 * @param rhs reference to existing Buffer_2D object
                 */
                Buffer_2D(const Buffer_2D<data_type> & rhs):
                    Buffer<data_type>(rhs),
                    buffer(rhs.buffer),
                    num_rows(rhs.num_rows),
                    num_cols(rhs.num_cols),
                    excess_samples(rhs.excess_samples)
                    {}

                
                /**
                 * @brief Assignment operator
                 * 
                 * @param rhs reference to another Buffer_2D class
                 * @return Buffer_2D& 
                 */
                Buffer_2D & operator=(const Buffer_2D<data_type> & rhs){
                    if(this != & rhs) {
                        Buffer<data_type>::operator=(rhs);

                        buffer = rhs.buffer;
                        num_rows = rhs.num_rows;
                        num_cols = rhs.num_cols;
                        excess_samples = rhs.excess_samples;
                    }

                    return * this;
                }
                
                /**
                 * @brief Destructor for Buffer_2D object
                 * 
                 */
                virtual ~Buffer_2D() {}

                
                /**
                 * @brief Reconfigure the 2D buffer object to have a specified number of rows,columns, and excess samples
                 * 
                 * @param rows 
                 * @param cols 
                 * @param excess 
                 * @param reset - resets the vector to all zeros on True. On False, only resets if the size has changed
                 */
                void reconfigure(size_t rows, size_t cols,size_t excess = 0, bool reset = true){
                    
                    //check to see if the dimmensions are new. Only compute a new buffer if the dimmensions or excess has changed
                    if (reset || (rows != num_rows) || (cols != num_cols) || (excess != excess_samples))
                    {
                        //initialize a new buffer with the correct dimmensions
                        buffer = std::vector<std::vector<data_type>>(rows,std::vector<data_type>(cols,0));
                        num_rows = rows;
                        num_cols = cols;
                        excess_samples = excess;
                    }
                }
                
                /**
                 * @brief prints out a 1d buffer
                 * 
                 * @param buffer_to_print a reference to the buffer(vector) to be printed
                 */
                void print_1d_buffer_preview(std::vector<data_type> & buffer_to_print){
                    //declare variable to keep track of how many samples to print out (limited to the first 5 and the last sample)
                    size_t samples_to_print;
                    if (buffer_to_print.size() > 5){
                        samples_to_print = 5;
                    }
                    else if (buffer_to_print.size() > 0)
                    {
                        samples_to_print = buffer_to_print.size();
                    }
                    
                    else{
                        std::cerr << "Buffer_2D::print_1d_buffer_preview: buffer to print is empty" << std::endl;
                        return;
                    }

                    //print out the first five samples
                    for (size_t i = 0; i < samples_to_print; i++)
                    {
                       std::cout << buffer_to_print[i] << ", ";
                    }

                    //if there are more than 5 samples, print out the last sample from the vector as well
                    if(buffer_to_print.size() > 5){
                        std::cout << "...., " << buffer_to_print.back() << std::endl;
                    }
                    else{
                        std::cout << std::endl;
                    }
                    
                    return;
                }
                             
                
                /**
                 * @brief load data from a 1D vector into the buffer
                 * 
                 * @param data_to_load (1D vector) the data to load into the buffer
                 * @param copy_until_buffer_full (on true) continuously copies data from the vector into the buffer until
                 * the buffer is full or until the excess samples is reached, even if multiple copies of the data are made
                 * (on false) inserts up to only 1 copy of the data into the buffer (assumes buffer has already been initialized)
                 */
                void load_data_into_buffer(std::vector<data_type> & data_to_load, bool copy_until_buffer_full = true){

                    //get the number of samples in the data_to_load
                    size_t m = data_to_load.size(); //rows
                    
                    
                    //determine the maximum sample number for the initialized array
                    size_t samples_to_load = num_rows * num_cols;

                    //initialize variables for reshaping
                    size_t from_idx;
                    size_t to_r;
                    size_t to_c;

                    for (size_t i = 0; i < samples_to_load; i++)
                    {
                        //index in data_to_load
                        from_idx = i % m;

                        //indicies in current array
                        to_r = i/num_cols;
                        to_c = i % num_cols;

                        if (i >= m)
                        {
                            if (copy_until_buffer_full)
                            {
                                buffer[to_r][to_c] = data_to_load[from_idx];
                            }
                            else{
                                buffer[to_r][to_c] = 0;
                            }
                        }
                        else{
                            buffer[to_r][to_c] = data_to_load[from_idx];
                        }
                    }
                }
                
                
                /**
                 * @brief load data from a 2D vector into the buffer
                 * 
                 * @param data_to_load (2D vector) the data to load into the buffer
                 * @param copy_until_buffer_full (on true) continuously copies data from the vector into the buffer until
                 * the buffer is full or until the excess samples is reached, even if multiple copies of the data are made
                 * (on false) inserts up to only 1 copy of the data into the buffer. (Assumes buffer has already been initialized)
                 */
                void load_data_into_buffer(std::vector<std::vector<data_type>> & data_to_load, bool copy_until_buffer_full = true){

                    //get dimmensions of data_to_load array
                    size_t m = data_to_load.size(); //rows
                    size_t n = data_to_load[0].size(); //cols
                    
                    
                    //determine the maximum sample number depending on copy behavior
                    size_t samples_to_load = num_rows * num_cols;

                    //initialize variables for reshaping
                    size_t from_r;
                    size_t from_c;
                    size_t to_r;
                    size_t to_c;

                    for (size_t i = 0; i < samples_to_load; i++)
                    {
                        //indicies in data_to_load
                        from_r = i/n;
                        from_c = i % n;

                        //indicies in current array
                        to_r = i/num_cols;
                        to_c = i % num_cols;

                        if (from_r >= m)
                        {
                            if (copy_until_buffer_full)
                            {
                                buffer[to_r][to_c] = data_to_load[from_r % m][from_c];
                            }
                            else{
                                buffer[to_r][to_c] = 0;
                            }
                        }
                        else{
                            buffer[to_r][to_c] = data_to_load[from_r][from_c];
                        }
                    }
                }

                /**
                 * @brief print a prevew of the buffer (1st 3 rows, 1st 5 columns, last row, last column)
                 * 
                 */
                virtual void print_preview(){
                    size_t rows_to_print;
                    if (buffer.size() > 3){
                        rows_to_print = 3;
                    }
                    else if (buffer.size() > 0)
                    {
                        rows_to_print = buffer.size();
                    }
                    
                    else{
                        std::cerr << "Buffer_2D::print_preview: buffer to print is empty" << std::endl;
                    }

                    //print out the first three samples
                    for (size_t i = 0; i < rows_to_print; i++)
                    {
                        print_1d_buffer_preview(buffer[i]);
                    }

                    //if there are more than 5 samples, print out the last sample from the vector as well
                    if(buffer.size() > 5){
                        std::cout << "\t...\t" << std::endl;
                        print_1d_buffer_preview(buffer.back());
                    }
                    else{
                        std::cout << std::endl;
                    }
                    return;
                }

                /**
                 * @brief Function to load data from a file into the current buffer
                 * (assumes that the buffer has already been initialized and the read
                 * file stream is already initialized.)
                 * 
                 */
                virtual void import_from_file(){
                    
                    if (Buffer<data_type>::buffer_init_status == true){
                        //save the data from the file into a vector
                        std::vector<data_type> data = Buffer<data_type>::load_data_from_read_file();
                        
                        //load it into the file
                        load_data_into_buffer(data,false);
                    }
                    else {
                        std::cerr << "Buffer_2D::import_from_file: attempted to import from file when buffer wasn't initialized" <<std::endl;
                    }
                    
                }

                /**
                 * @brief gets data from the file, initializes the 
                 * buffer for a given number of columns, and loads the data into the buffer
                 * (note: assumes read file stream has already been initialized)
                 * 
                 * @param desired_num_cols the number of columns in the buffer (automatically 
                 * computes the number of rows))
                 */
                void import_from_file(double desired_num_cols){
                        //save the data from the file into a vector
                        std::vector<data_type> data = Buffer<data_type>::load_data_from_read_file();
                        
                        double num_samples = static_cast<double>(data.size());

                        num_cols = static_cast<int>(desired_num_cols);
                        num_rows = static_cast<int>(std::ceil(num_samples/desired_num_cols));
                        excess_samples = static_cast<int>(num_samples) % num_cols;

                        buffer = std::vector<std::vector<data_type>>(num_rows,std::vector<data_type>(num_cols));
                        
                        //load it into the file
                        load_data_into_buffer(data,false);

                        return;
                }

                /**
                 * @brief Save the current buffer to the file (assumes that the write file stream is already open)
                 * 
                 */
                virtual void save_to_file(){
                    //the out file stream must already be open when the function is called
                    if(Buffer<data_type>::write_file_stream -> is_open()){
                        //save all of the rows except for the last one
                        for (size_t i = 0; i < num_rows - 1; i++)
                        {
                            Buffer<data_type>::write_file_stream -> write((char*) &buffer[i].front(), buffer[i].size() * sizeof(data_type));
                        }
                        //for the last row, since there may be excess samples in the buffer, only save those pertaining to a chirp
                        Buffer<data_type>::write_file_stream -> write((char*) &buffer.back().front(), (num_cols - excess_samples) * sizeof(data_type));

                    }
                    else{
                        std::cerr << "Buffer_2D::save_to_file: write_file_stream not open" << std::endl;
                    }
                }

        }; // end of Buffer_2D class

        /**
         * @brief Buffer_1D a 1D buffer that stores data in a buffer and allows easy read/write integration witha file
         * 
         * @tparam data_type: the type of data that the buffer stores 
         */
        template<typename data_type>
        class Buffer_1D : public Buffer<data_type>{
            public:
            //variables
                //a vector to store things in
                std::vector<data_type> buffer;

                //keep track of the size of the buffer
                size_t num_samples;

                //keep track of how many excess samples are unused in the buffer
            //funcions

                /**
                 * @brief Construct a new Buffer_1D object
                 * 
                 */
                Buffer_1D() : Buffer<data_type>() {}

                /**
                 * @brief Construct a new Buffer_1D object
                 * 
                 * @param debug the desired debug setting
                 */
                Buffer_1D(bool debug) : Buffer<data_type>(false,debug) {}

                /**
                 * @brief Construct a new Buffer_1D object
                 * 
                 * @param samples the number of samples that the buffer stores
                 * @param debug the desired debug setting
                 */
                Buffer_1D(size_t samples, bool debug = false) 
                    : Buffer<data_type>(true,debug),
                    buffer(samples,data_type(0)), //initializing the buffer with all zeros
                    num_samples(samples) {}

                /**
                 * @brief Construct a new Buffer_1D object, initialize with an existing vector
                 * 
                 * @param data_to_load reference to vector containing data to load
                 */
                Buffer_1D(std::vector<data_type> & data_to_load):
                    Buffer<data_type>(),
                    buffer(data_to_load),
                    num_samples(data_to_load.size())
                    {}
                
                /**
                 * @brief Construct a new Buffer_1D object- COPY Constructor
                 * 
                 * @param rhs reference to existing Buffer_1D object
                 */
                Buffer_1D(const Buffer_1D<data_type> & rhs):
                    Buffer<data_type>(rhs),
                    buffer(rhs.buffer),
                    num_samples(rhs.num_samples)
                    {}

                
                /**
                 * @brief Assignment operator
                 * 
                 * @param rhs reference to another Buffer_1D class
                 * @return Buffer_1D& 
                 */
                Buffer_1D & operator=(const Buffer_1D<data_type> & rhs){
                    if(this != & rhs) {
                        Buffer<data_type>::operator=(rhs);

                        buffer = rhs.buffer;
                        num_samples = rhs.num_samples;
                    }

                    return * this;
                }


                /**
                 * @brief Destroy the Buffer_1D object
                 * 
                 */
                virtual ~Buffer_1D() {}

                /**
                 * @brief print a preview of the buffer (assumes buffer already has samples in it)
                 * 
                 */
                virtual void print_preview() {
                    
                    //declare variable to keep track of how many samples to print out (limited to the first 5 and the last sample)
                    size_t samples_to_print;
                    if (buffer.size() > 5){
                        samples_to_print = 5;
                    }
                    else if (buffer.size() > 0)
                    {
                        samples_to_print = buffer.size();
                    }
                    
                    else{
                        std::cerr << "Buffer_1D::print_1d_buffer_preview: buffer to print is empty" << std::endl;
                        return;
                    }

                    //print out the first five samples
                    for (size_t i = 0; i < samples_to_print; i++)
                    {
                       std::cout << buffer[i] << ", ";
                    }

                    //if there are more than 5 samples, print out the last sample from the vector as well
                    if(buffer.size() > 5){
                        std::cout << "...., " << buffer.back() << std::endl;
                    }
                    else{
                        std::cout << std::endl;
                    }
                }

                
                void load_data_into_buffer(std::vector<data_type> & data_to_load){

                    //get the number of samples in the data_to_load
                    num_samples = data_to_load.size(); //rows
                    
                    //save the buffer
                    buffer = data_to_load;
                }
                
                /**
                 * @brief imports data from an already opened read_file_stream and loads it into the buffer,
                 * sets the buffer to be equal to the data in the buffer and sets num_samples to the number
                 * of samples imported from the file, Note: read file stream is closed after data is imported
                 * 
                 */
                virtual void import_from_file(){
                    //load the data from the read file
                    buffer = Buffer<data_type>::load_data_from_read_file();
                    num_samples = buffer.size();
                }

                /**
                 * @brief save the current buffer to the write_file_sream (assumes write file stream
                 * is already open)
                 * 
                 */
                virtual void save_to_file(){
                    if(Buffer<data_type>::write_file_stream -> is_open()){
                        //save all of the rows except for the last one
                        Buffer<data_type>::write_file_stream -> write((char*) &buffer.front(), buffer.size() * sizeof(data_type));
                    }
                    else{
                        std::cerr << "Buffer_1D::save_to_file: write_file_stream not open" << std::endl;
                    }
                }

                /**
                 * @brief Set size of the buffer and initialize a buffer of that size
                 * 
                 * @param num_samps the number of samples for the buffer to store
                 */
                void set_buffer_size(size_t num_samps){
                    //initialize the buffer with all zeros
                    buffer = std::vector<data_type>(num_samps,data_type(0));
                    num_samples = num_samps;
                }

                /**
                 * @brief Add an element to the end of the buffer and increment the number of samples
                 * 
                 * @param element the element to add
                 */
                void push_back(data_type element){
                    num_samples += 1;
                    buffer.push_back(element);
                }

                /**
                 * @brief Removes the item at the specified index from the buffer
                 * 
                 * @param index the index within the buffer to remove the object at
                 */
                void remove_item(size_t index){
                    if (num_samples > 0)
                    {
                        buffer.erase(buffer.begin() + index);
                        num_samples -= 1;
                    }
                }

                /**
                 * @brief Clear the buffer and reset the size to zero
                 * 
                 */
                void clear(){
                    num_samples = 0;
                    buffer.clear();
                }

                /**
                 * @brief Set the provided indicies in the buffer to the specified value
                 * 
                 * @param value the value to set
                 * @param start_idx start index
                 * @param end_idx end index
                 */
                void set_val_at_indicies(data_type value,size_t start_idx, size_t end_idx){
                    for (size_t i = start_idx; i < end_idx; i++)
                    {
                        buffer[i] = value;
                    }
                    
                }

                /**
                 * @brief find the indicies that have the specific value
                 * 
                 * @param value the value to search for
                 * @param start_idx the index to start the search at (default to zero)
                 * @param data_sorted boolean, true if data is sorted
                 * @return std::vector<size_t> vector of indicies that the value is found at
                 */
                std::vector<size_t> find_indicies_with_value(data_type value, size_t start_idx = 0, bool data_sorted = true){
                    
                    //vector to store the indicies that have the desired value
                    std::vector<size_t> index_list;

                    //status variables
                    bool value_found = false;
                    bool end_search = false;

                    //perform the search
                    for (size_t i = start_idx; i < num_samples && !end_search; i++)
                    {   
                        //start searching until an index with the desired value is found
                        if (buffer[i] == value)
                        {
                            index_list.push_back(i);
                            value_found = true;
                        }
                        //if an index with the value has already been found
                        else if (value_found && data_sorted)
                        {
                            end_search = true;
                        }
                    }
                    return index_list;
                }

        }; // end Buffer_1D class
    
        /**
         * @brief A data 2D buffer, specifically designed for radar on the USRP 
         * 
         * @tparam data_type creates a RADAR Buffer of type std::complex<data_type>
         */
        template<typename data_type>
        class FMCW_Buffer : public Buffer_2D<std::complex<data_type>>{
            public:
                //variables
                size_t num_chirps;
                size_t samples_per_chirp;

                //functions
                    
                    
                    /**
                     * @brief Construct a new fmcw buffer object
                     * 
                     */
                    FMCW_Buffer(): Buffer_2D<std::complex<data_type>>(){}
                    
                    /**
                     * @brief Construct a new fmcw buffer object
                     * 
                     * @param debug the desired debug setting
                     */
                    FMCW_Buffer(bool debug): Buffer_2D<data_type>(debug){}
                    
                    /**
                     * @brief Construct a new RADAR buffer object (initialized)
                     * 
                     * @param desired_samples_per_buff the number of samples per buffer (for the USRP)
                     * @param required_samples_per_chirp the number of samples in a single chirp
                     * @param desired_num_chirps the number of chirps the buffer should contain
                     * @param debug the desired debug setting (optional)
                     */
                    FMCW_Buffer(
                        size_t desired_samples_per_buff,
                        size_t required_samples_per_chirp,
                        size_t desired_num_chirps,
                        bool debug = false) 
                        : Buffer_2D<std::complex<data_type>>(debug){
                            configure_fmcw_buffer(
                                desired_samples_per_buff,
                                required_samples_per_chirp,
                                desired_num_chirps,
                                true);
                        }
                    

                    /**
                     * @brief Construct a new fmcw buffer object - COPY Constructor
                     * 
                     * @param rhs reference to another FMCW_Buffer object to copy
                     */
                    FMCW_Buffer(const FMCW_Buffer<data_type> & rhs):
                        Buffer_2D<data_type>(rhs),
                        num_chirps(rhs.num_chirps),
                        samples_per_chirp(rhs.samples_per_chirp)
                        {}

                    /**
                     * @brief Assignment Operator
                     * 
                     * @param rhs the FMCW Buffer object who's values we wish to assign to the current FMCW Buffer
                     * @return FMCW_Buffer& the current FMCW Buffer
                     */
                    FMCW_Buffer & operator=(const FMCW_Buffer<data_type> & rhs){
                        if(this != & rhs) {
                            Buffer_2D<data_type>::operator=(rhs);

                            num_chirps = rhs.num_chirps;
                            samples_per_chirp = rhs.samples_per_chirp;
                        }

                        return * this;
                    }

                    /**
                     * @brief Destroy the fmcw buffer object
                     * 
                     */
                    virtual ~FMCW_Buffer() {}

                    /**
                     * @brief configures a Buffer_2D to be able to operate as a buffer used by the RADAR radar,
                     * and initializes a buffer (vector) of the correct dimensions
                     * 
                     * @param desired_samples_per_buff the number of samples in a buffer
                     * @param required_samples_per_chirp the number of samples per chirp
                     * @param desired_num_chirps the number of chirps
                     * @param reset_buffer automatically resets the buffer if true. If false, function will check dimmensions and parameters to see if they have changed. Buffer will be reset only if the dimmensions or parameters have changed
                     */
                    void configure_fmcw_buffer(
                        size_t desired_samples_per_buff,
                        size_t required_samples_per_chirp,
                        size_t desired_num_chirps,
                        bool reset_buffer = false)
                    {   
                        //check to make sure that the dimmensions have changed. Only re-configure the buffer if dimmensions have changed (unless reset buffer is true)
                        if ( reset_buffer ||
                            (desired_samples_per_buff != Buffer_2D<std::complex<data_type>>::num_cols) ||
                            (required_samples_per_chirp != samples_per_chirp) ||
                            (desired_num_chirps != num_chirps))
                        {
                            num_chirps = desired_num_chirps;
                            samples_per_chirp = required_samples_per_chirp;
                            Buffer_2D<std::complex<data_type>>::num_cols = desired_samples_per_buff;

                            if (desired_samples_per_buff == required_samples_per_chirp){
                                Buffer_2D<std::complex<data_type>>::num_rows = desired_num_chirps;
                                Buffer_2D<std::complex<data_type>>::excess_samples = 0;
                            }
                            else if (((desired_num_chirps * required_samples_per_chirp) % desired_samples_per_buff) == 0){
                                Buffer_2D<std::complex<data_type>>::num_rows = (desired_num_chirps * required_samples_per_chirp) / desired_samples_per_buff;
                                Buffer_2D<std::complex<data_type>>::excess_samples = 0;   
                            }
                            else
                            {
                                Buffer_2D<std::complex<data_type>>::num_rows = ((desired_num_chirps * required_samples_per_chirp) / desired_samples_per_buff) + 1;
                                Buffer_2D<std::complex<data_type>>::excess_samples = (Buffer_2D<std::complex<data_type>>::num_rows * desired_samples_per_buff) - (desired_num_chirps * required_samples_per_chirp);
                            }

                            Buffer_2D<std::complex<data_type>>::buffer = std::vector<std::vector<std::complex<data_type>>>(Buffer_2D<std::complex<data_type>>::num_rows,std::vector<std::complex<data_type>>(Buffer_2D<std::complex<data_type>>::num_cols));
                            Buffer<std::complex<data_type>>::buffer_init_status = true;
                        }                        
                    }

                    /**
                     * @brief loads data from a vector into the initizlized RADAR buffer, making copies of the 
                     * vector as necessary until the buffer is full or the number of excess samples has been
                     * reached
                     * 
                     * @param chirp a vector containing the samples for multiple chirps
                     */
                    void load_chirp_into_buffer(std::vector<std::complex<data_type>> & chirp){
                        Buffer_2D<std::complex<data_type>>::load_data_into_buffer(chirp,true);
                    }

                    /**
                     * @brief loads data from a 2D vector containing multiple chirps into the 
                     * initialized FMCW buffer. Only 1 copy of the 2D vector is loaded
                     * 
                     * @param chirps a 2D vector containing the samples for multiple chirps
                     */
                    void load_chirps_into_buffer(std::vector<std::vector<std::complex<data_type>>> & chirps){
                        Buffer_2D<std::complex<data_type>>::load_data_into_buffer(chirps,false);
                    }
        };


        /**
         * @brief A customized 1D buffer, specifically designed for saving and performing estimation for a specific parameter
         * 
         * @tparam data_type 
         */
        template<typename data_type>
        class Parameter_Estimation_Buffer: public Buffer_1D<data_type>
        {
        private:
            
            //parameter to track the number of estimates that the buffer contains
            size_t num_estimates;
            
            //parameters to store the mean and variance of the parameter estimates
            data_type mean;
            data_type variance;

            //status variable to track if the parameter estimation buffer is full
            bool buffer_full;

            //parameters to detect randomization
            data_type randomization_threshold;
            size_t randomization_num_samples;
            bool randomization_detected;
            bool randomization_detection_configured;
        public:

            /**
             * @brief Create an empty parameter estimation buffer
             * 
             */
            Parameter_Estimation_Buffer() : 
                Buffer_1D<data_type>(),
                num_estimates(0),
                mean(0),
                variance(0),
                buffer_full(false),
                randomization_threshold(0.0),
                randomization_num_samples(0),
                randomization_detected(false),
                randomization_detection_configured(false)
                {}
            
            /**
             * @brief Construct a new Parameter_Estimation_Buffer object
             * 
             * @param samples The number of estimates for the parameter estimation buffer to store
             * @param debug debug status
             */
            Parameter_Estimation_Buffer(size_t samples, bool debug=false):
                Buffer_1D<data_type>(samples,debug),
                num_estimates(0),
                mean(0),
                variance(0),
                buffer_full(0),
                randomization_threshold(0.0),
                randomization_num_samples(0),
                randomization_detected(false),
                randomization_detection_configured(false)
                {}

            
            /**
             * @brief Construct a new Parameter_Estimation_Buffer object - COPY CONSTRUCTOR
             * 
             * @param rhs reference to another parameter estimation buffer object
             */
            Parameter_Estimation_Buffer(const Parameter_Estimation_Buffer<data_type> & rhs):
                Buffer_1D<data_type>(rhs),
                num_estimates(rhs.num_estimates),
                mean(rhs.mean),
                variance(rhs.variance),
                buffer_full(rhs.buffer_full),
                randomization_threshold(rhs.randomization_threshold),
                randomization_num_samples(rhs.randomization_num_samples),
                randomization_detected(rhs.randomization_detected),
                randomization_detection_configured(rhs.randomization_detection_configured)
                {}

            /**
             * @brief Assigment Operator
             * 
             * @param rhs object to make assignment from
             * @return Parameter_Estimation_Buffer& 
             */
            Parameter_Estimation_Buffer & operator=(const Parameter_Estimation_Buffer<data_type> & rhs){
                if(this != & rhs){
                    Buffer_1D<data_type>::operator=(rhs);
                    num_estimates = rhs.num_estimates;
                    mean = rhs.mean;
                    variance = rhs.variance;
                    buffer_full = rhs.buffer_full;
                    randomization_threshold = rhs.randomization_threshold;
                    randomization_num_samples = rhs.randomization_num_samples;
                    randomization_detected = rhs.randomization_detected;
                    randomization_detection_configured = rhs.randomization_detection_configured;
                }

                return * this;
            }
            
            /**
             * @brief Destroy Parameter Estimation Buffer
             * 
             */
            virtual ~Parameter_Estimation_Buffer() {}


            /**
             * @brief Reset the parameter estimation buffer estimates, and all of the samples in the buffer to zero
             * 
             * @param samples the number of samples for the buffer to store. If zero, will use the current number of samples. Defaults to zero
             */
            void reset(size_t samples = 0){

                //reset the buffer to be the specified number of samples
                if(samples > 0){
                    Buffer_1D<data_type>::set_buffer_size(samples);
                }
                
                //reset the other variables
                num_estimates = 0;
                mean = 0;
                variance = 0;
                buffer_full = 0;
                randomization_threshold = 0.0;
                randomization_num_samples = 0;
                randomization_detected = false;
                randomization_detection_configured = false;
            }

            /**
             * @brief Loads new parameter estimates into the buffer, provided that there is room in the buffer
             * 
             * @param estimates a vector containing the new estimates to include
             */
            void load_estimates(std::vector<data_type> & estimates){
                if (not buffer_full)
                {
                    for (size_t i = 0; i < estimates.size(); i++)
                    {
                        //add the estimate into the buffer
                        Buffer_1D<data_type>::buffer[num_estimates] = estimates[i];

                        //incrememtn the number of estimates
                        num_estimates++;

                        //check to make sure that the buffer still has room
                        if (num_estimates == Buffer_1D<data_type>::num_samples)
                        {
                            buffer_full = true;
                            break;
                        }
                    }
                }
                update_statistics();
                return;
            }

            /**
             * @brief Loads new parameter estimates into the buffer, provided that there is room in the buffer
             * 
             * @param estimates a 1D Buffer containing the new estimates to include
             */
            void load_estimates(Buffer_1D<data_type> & estimates){
                if (not buffer_full)
                {
                    for (size_t i = 0; i < estimates.num_samples; i++)
                    {
                        //add the estimate into the buffer
                        Buffer_1D<data_type>::buffer[num_estimates] = estimates.buffer[i];

                        //incrememtn the number of estimates
                        num_estimates++;

                        //check to make sure that the buffer still has room
                        if (num_estimates == Buffer_1D<data_type>::num_samples)
                        {
                            buffer_full = true;
                            break;
                        }
                    }
                }
                update_statistics();
                return;
            }

            /**
             * @brief Load a single new estimate into the parameter estimation buffer
             * 
             * @param estimate the parameter estimate to load
             */
            void load_estimate(data_type estimate){
                if (not buffer_full){
                    //add the estimate to the buffer
                    Buffer_1D<data_type>::buffer[num_estimates] = estimate;
                    num_estimates ++;

                    //check to make sure that the buffer still has room
                    if (num_estimates == Buffer_1D<data_type>::num_samples)
                    {
                        buffer_full = true;
                    }
                }
                update_statistics();
            }

            /**
             * @brief Load in new parameter estimates, but from intercepts. The parameter estimation buffer will use the difference between consecutive intercepts as the estimate for the parameter
             * 
             * @param intercepts a vector of intercepts to use when computing the parameter estimates
             * @param use_previous_intercept On True, will use the provided previous intercept to compute the estimation for the first parameter estimate. Defaults to False
             * @param previous_intercept A previuos intercept for the (n=-1) case which will be used to compute the difference between (n=0 and n=-1). Only used when use_previous_intercept is true 
             */
            void load_estimates_from_intercepts(
                    std::vector<data_type> & intercepts,
                    bool use_previous_intercept = false,
                    data_type previous_intercept = 0)
            {   

                if(not buffer_full){
                        //compute the number of estimates
                    size_t new_estimates;
                    if (use_previous_intercept)
                    {
                        new_estimates = intercepts.size();
                    }
                    else{
                        new_estimates = intercepts.size() - 1;
                    }
                    
                    //initialize a vector to store the estimates in
                    std::vector<data_type> estimates(new_estimates);
                    
                    //compute the estimates from the intercepts
                    if(use_previous_intercept){
                        estimates[0] = intercepts[0] - previous_intercept;
                        for (size_t i = 1; i < new_estimates; i++){
                            estimates[i] = intercepts[i] - intercepts[i - 1];
                        }
                    }
                    else{
                        for (size_t i = 0; i < new_estimates; i++){
                            estimates[i] = intercepts[i + 1] - intercepts[i];
                        }
                    }

                    load_estimates(estimates);
                }
                //don't need to update statistics because load_estimates already does it
                return;
            }

            /**
             * @brief Load in new parameter estimates, but from intercepts. The parameter estimation buffer will use the difference between consecutive intercepts as the estimate for the parameter
             * 
             * @param intercepts a 1D Buffer of intercepts to use when computing the parameter estimates
             * @param use_previous_intercept On True, will use the provided previous intercept to compute the estimation for the first parameter estimate. Defaults to False
             * @param previous_intercept A previuos intercept for the (n=-1) case which will be used to compute the difference between (n=0 and n=-1). Only used when use_previous_intercept is true 
             */
            void load_estimates_from_intercepts(
                    Buffer_1D<data_type> & intercepts,
                    bool use_previous_intercept = false,
                    data_type previous_intercept = 0)
            {   

                if(not buffer_full){
                        //compute the number of estimates
                    size_t new_estimates;
                    if (use_previous_intercept)
                    {
                        new_estimates = intercepts.num_samples;
                    }
                    else{
                        new_estimates = intercepts.num_samples - 1;
                    }
                    
                    //initialize a vector to store the estimates in
                    std::vector<data_type> estimates(new_estimates);
                    
                    //compute the estimates from the intercepts
                    if(use_previous_intercept){
                        estimates[0] = intercepts.buffer[0] - previous_intercept;
                        for (size_t i = 1; i < new_estimates; i++){
                            estimates[i] = intercepts.buffer[i] - intercepts.buffer[i - 1];
                        }
                    }
                    else{
                        for (size_t i = 0; i < new_estimates; i++){
                            estimates[i] = intercepts.buffer[i + 1] - intercepts.buffer[i];
                        }
                    }

                    load_estimates(estimates);
                }
                //don't need to update statistics because load_estimates already does it
                return;
            }

            /**
             * @brief Load a new estimate into the parameter estimation buffer given a two intercepts
             * 
             * @param intercept the intercept at n=0
             * @param previous_intercept the intercept at n=-1
             */
            void load_estimate_from_intercept(
                data_type intercept,
                data_type previous_intercept)
            {
                if (not buffer_full)
                {
                    load_estimate(intercept - previous_intercept);
                }
                //don't need to update statistics because load_estimates already does it
                return;
            }

            /**
             * @brief Removes outliers from the parameter estimates by looking for samples that are 1.5 * IQR away from the Q3 or Q1 samples (quartiles)
             * 
             * @param min_samples the minimum number of samples required before outliers can be removed
             */
            void remove_outliers(size_t min_samples = 20){
                    if (num_estimates > 20)
                    {
                        //sort the samples
                        std::sort(Buffer_1D<data_type>::buffer.begin(), Buffer_1D<data_type>::buffer.begin() + num_estimates);

                        //compute the IQR range
                        size_t Q1_sample = static_cast<size_t>(std::ceil(0.25 * static_cast<double>(num_estimates)));
                        size_t Q3_sample = static_cast<size_t>(std::floor(0.75 * static_cast<double>(num_estimates)));

                        data_type IQR = Buffer_1D<data_type>::buffer[Q3_sample] - Buffer_1D<data_type>::buffer[Q1_sample];
                        data_type upper_fence = Buffer_1D<data_type>::buffer[Q3_sample] + 1.5 * IQR;
                        data_type lower_fence = Buffer_1D<data_type>::buffer[Q1_sample] - 1.5 * IQR;

                        //sweep through the samples and remove any outliers
                        size_t i = 0;
                        data_type val;
                        while (i < num_estimates)
                        {   
                            val = Buffer_1D<data_type>::buffer[i];
                            if ((val < lower_fence) || (val > upper_fence))
                            {
                                //remove the outlier
                                Buffer_1D<data_type>::remove_item(i);
                                num_estimates -= 1;
                                buffer_full = false; //buffer would no longer be full
                            }else
                            {
                                i++;
                            }
                        }
                        //update the statistics after the outliers have been removed
                        update_statistics();
                    }
                return;
            }

            /**
             * @brief Configure parameter randomization detection
             * 
             * @param threshold the threshold to use when detecting randomization
             * @param num_samples the number of samples to use when computing the variance for detecting parameter randomization
             */
            void configure_randomization_detection(data_type threshold, size_t num_samples)
            {
                randomization_threshold = threshold;
                randomization_num_samples = num_samples;
                randomization_detection_configured = true;
                randomization_detected = false;
            }
            
            /**
             * @brief Get the mean
             * 
             * @return mean 
             */
            data_type get_mean(){
                return mean;
            }

            /**
             * @brief Get the variance
             * 
             * @return variance 
             */
            data_type get_variance(){
                return variance;
            }

            /**
             * @brief Get the stdev
             * 
             * @return stdev
             */
            data_type get_stdev(){
                return std::sqrt(variance);
            }

            /**
             * @brief Get the buffer full status
             * 
             * @return true 
             * @return false 
             */
            bool get_buffer_full_status(){
                return buffer_full;
            }

            /**
             * @brief Get the randomization detected status
             * 
             * @return true parameter randomization detected
             * @return false parameter randomization not detected
             */
            bool get_randomization_detected(){
                if(randomization_detection_configured){
                    return randomization_detected;
                }else{
                    std::cerr << "Parameter_Estimation_Buffer::get_randomization_detected: randomization detection not yet enabled" << std::endl;
                    return false;
                }
            }

        private: //helper functions for computing statistics

            /**
             * @brief Update the mean and variance
             * 
             */
            void update_statistics(){
                //update the mean
                compute_mean();

                //update the variance
                compute_variance();

                //check for randomization
                if(randomization_detection_configured){
                    check_for_randomization();
                }
            }
            
            /**
             * @brief Compute the sample mean of the parameters
             * 
             */
            void compute_mean(){
                data_type sum = 0;
                for (size_t i = 0; i < num_estimates; i++)
                {
                    sum += Buffer_1D<data_type>::buffer[i];
                }
                mean = sum / static_cast<data_type>(num_estimates);
            }

            /**
             * @brief Compute the sample variance
             * 
             */
            void compute_variance(){
                data_type sum = 0;
                for (size_t i = 0; i < num_estimates; i++){
                    sum += std::pow((Buffer_1D<data_type>::buffer[i] - mean),2);
                }
                variance = sum / static_cast<data_type>(num_estimates);
            }

            /**
             * @brief If parameter randomization is configured, checks to see if parameter randomization is being employed
             * 
             */
            void check_for_randomization(){
                //ensure that randomization detection has been configured
                if(randomization_detection_configured){
                    //make sure that there are enough samples in the buffer
                    if (num_estimates >= randomization_num_samples)
                    {
                        //compute the variance of the specified number of samples
                        data_type sum = 0;
                        for (size_t i = 0; i < randomization_num_samples; i++){
                            sum += std::pow((Buffer_1D<data_type>::buffer[i] - mean),2);
                        }
                        data_type estimated_variance = sum / static_cast<data_type>(num_estimates);

                        //compare against the threshold
                        if(estimated_variance > randomization_threshold){
                            randomization_detected = true;
                        }else{
                            randomization_detected = false;
                        }
                    }else{
                        randomization_detected = false;
                    }
                    
                }else{
                    std::cerr << "Parameter_Estimation_Buffer::check_for_randomization: randomization detection not yet enabled" << std::endl;
                    randomization_detected = false;
                }
            }

        }; //end of Parameter_Estimation_Buffer class
        
    }//end of namespace
#endif