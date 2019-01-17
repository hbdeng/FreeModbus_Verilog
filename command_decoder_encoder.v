
/**
 * @file mb.v
 * @auther denghb
 * @breif verilog implementation of Modbus slaver protocal.
 * The Modbus protocal is ported from FreeModbus.
 * @version 1.1
**/


module command_decoder_encoder
(


    // inputs
    clk,
    rst_n,
    rxd,
    generator_is_running,
    wave_calculator_is_running,
    
    
    // outputs
    txd,
    do_start_generator,
    do_stop_generator,
    voltage1,
    voltage2,
    polar,
    settling_divisor,
    delay_divisor,
    wave_points,
    steps,
    down_sampling_divisor,
    do_start_wave_calculator
    
    
);


    // inputs
    input           clk;
    input           rst_n;
    input           rxd;
    input           generator_is_running;
    input           wave_calculator_is_running;
    
    
    // outputs
    output          txd;
    output          do_start_generator;
    output          do_stop_generator;
    output  [15:0]  voltage1;
    output  [15:0]  voltage2;
    output  [15:0]  polar;
    output  [15:0]  settling_divisor;
    output  [15:0]  delay_divisor;
    output  [15:0]  wave_points;
    output  [15:0]  steps;
    output  [15:0]  down_sampling_divisor;
    output          do_start_wave_calculator;
    
    
    // outputs' types
    wire            txd;
    reg             do_start_generator;
    reg             do_stop_generator;
    reg     [15:0]  voltage1;
    reg     [15:0]  voltage2;
    reg     [15:0]  polar;
    reg     [15:0]  settling_divisor;
    reg     [15:0]  delay_divisor;
    reg     [15:0]  wave_points;
    reg     [15:0]  steps;
    reg     [15:0]  down_sampling_divisor;
    reg             do_start_wave_calculator;
    
    
    // internal variables' types
    wire    [15:0]  baud_divisor;
    reg     [79:0]  decoder_shift_reg;
    reg     [79:0]  command_decoded;
    wire    [7:0]   command_decoded_initial;
    wire    [31:0]  command_decoded_function;
    wire    [31:0]  command_decoded_data;
    wire    [7:0]   command_decoded_terminator;
    reg             do_read_char;
    wire            do_force_break;
    wire    [7:0]   rx_data;
    wire            rx_char_ready;
    wire            break_detect;
    wire            receiving_start;
    wire            terminator_encountered;
    wire            receiving_complete;
    reg     [15:0]  voltage1_in;
    reg     [15:0]  voltage2_in;
    reg     [15:0]  polar_in;
    reg     [15:0]  settling_divisor_in;
    reg     [15:0]  delay_divisor_in;
    reg     [15:0]  wave_points_in;
    reg     [15:0]  steps_in;
    reg     [15:0]  down_sampling_divisor_in;
    wire            voltage1_changed;
    wire            voltage2_changed;
    wire            polar_changed;
    wire            settling_divisor_changed;
    wire            delay_divisor_changed;
    wire            wave_points_changed;
    wire            steps_changed;
    wire            down_sampling_divisor_changed;
    wire            parameters_changed;
    reg             wave_ready;
    wire    [80*11-1:0] command_encoded;
    reg     [80*11-1:0] command_encoded_reg;
    reg     [7:0]   tx_data;
    reg             do_send_char;
    wire            tx_shift_empty;
    wire            tx_ready;
    reg             do_start_status_transmitting;
    reg             status_transmitting_in_process;
    reg     [7:0]   encoder_counter;
    reg             encoder_counter_is_running;
    wire            encoder_counter_is_zero;
    wire    [4:0]   error_flag;
    wire            is_decoder_key_word_error;
    reg             is_decoder_command_error;
    reg             decoder_framing_error;
    wire    [3:0]   UART_error_flag;
    wire            clear_UART_error_flag;
    wire            clear_error_flag;
    
    
    // parameters
    
    
    // functions
    function    [3:0]   decoder_table;
        input   [7:0]   ascii_code;
        begin
            if (ascii_code < 8'h3A && ascii_code >= 8'h30)
                decoder_table = ascii_code[3:0];
            else if (ascii_code < 8'h47 && ascii_code > 8'h40)
                decoder_table = ascii_code[3:0] + 4'd9;
            else
                decoder_table = 4'd0;
        end
    endfunction
    function    [15:0]   decoder_integar;
        input   [31:0]   ascii_integar;
        begin
            decoder_integar[3:0] = decoder_table(ascii_integar[7:0]);
            decoder_integar[7:4] = decoder_table(ascii_integar[15:8]);
            decoder_integar[11:8] = decoder_table(ascii_integar[23:16]);
            decoder_integar[15:12] = decoder_table(ascii_integar[31:24]);
        end
    endfunction
    function    [7:0]   encoder_table;
        input   [3:0]   hex_code;
        begin
            if (hex_code < 10)
                encoder_table = hex_code + 8'h30;
            else
                encoder_table = hex_code + 8'h37;
        end
    endfunction
    function    [31:0]  encoder_integar;
        input   [15:0]  hex_integar;
        begin
            encoder_integar[7:0] = encoder_table(hex_integar[3:0]);
            encoder_integar[15:8] = encoder_table(hex_integar[7:4]);
            encoder_integar[23:16] = encoder_table(hex_integar[11:8]);
            encoder_integar[31:24] = encoder_table(hex_integar[15:12]);
        end
    endfunction
    
    
    // Baud rate is fixed to 9600 by assign baud_divisor 5207.
    assign baud_divisor = 15'd5207;
    
    
    // never transmitt break signal
    assign do_force_break = 0;
    
    
    // decoder shift register
    // initial: 8'h3A
    // function: 32bits
    // data: 32bits
    // terminator: 8'h3B
    assign receiving_start = rx_char_ready && !do_read_char && (rx_data == ":");
    assign terminator_encountered = rx_char_ready && !do_read_char && (rx_data == ";");
    assign receiving_complete = do_read_char && (rx_data == ";");
    assign {command_decoded_initial, command_decoded_function, command_decoded_data, command_decoded_terminator} = command_decoded;
    assign is_decoder_key_word_error = (command_decoded_initial != ":") && receiving_complete;
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            do_read_char <= 0;
        else if (do_read_char)
            do_read_char <= 0;
        else if (rx_char_ready)
            do_read_char <= 1;
    end
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            decoder_shift_reg <= 0;
        else if (rx_char_ready && !do_read_char)
            decoder_shift_reg <= {decoder_shift_reg[71:0], rx_data};
    end
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            command_decoded <= 0;
        else if (terminator_encountered)
            command_decoded <= {decoder_shift_reg[71:0], rx_data};
    end
    
    
    // command decoding
    // Command code:
    //  STRT: start generator
    //  STOP: stop generator
    //  RDST: transmit FPGA status to PC host
    //  VLT1: voltage 1
    //  VLT2: voltage 2
    //  POLA: wave polar
    //  SAMP: down sampling divisor
    //  STTM: settling time divisor
    //  DLTM: delay time divisor
    //  PNTS: points of one sweep, except for settling time and delay time
    //  STEP: steps
    //  default: is_decoder_command_error
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
        begin
            do_start_generator <= 0;
            do_stop_generator <= 0;
            do_start_status_transmitting <= 0;
            voltage1_in <= 32768;
            voltage2_in <= 32768;
            polar_in <= 0;
            down_sampling_divisor_in <= 10;
            settling_divisor_in <= 10;
            delay_divisor_in <= 10;
            wave_points_in <= 2048;
            steps_in <= 1;
            is_decoder_command_error <= 0;
        end
        else if (receiving_complete)
            case (command_decoded_function)
                "STRT":
                    if (!generator_is_running && !wave_calculator_is_running)
                        do_start_generator <= 1;
                "STOP":
                    if (generator_is_running)
                        do_stop_generator <= 1;
                "RDST":
                    if (!status_transmitting_in_process)
                        do_start_status_transmitting <= 1;
                "VLT1":
                    voltage1_in <= decoder_integar(command_decoded_data);
                "VLT2":
                    voltage2_in <= decoder_integar(command_decoded_data);
                "POLA":
                    polar_in <= decoder_integar(command_decoded_data);
                "STTM":
                    settling_divisor_in <= decoder_integar(command_decoded_data);
                "DLTM":
                    delay_divisor_in <= decoder_integar(command_decoded_data);
                "PNTS":
                    wave_points_in <= decoder_integar(command_decoded_data);
                "STEP":
                    steps_in <= decoder_integar(command_decoded_data);
                "SAMP":
                    down_sampling_divisor_in <= decoder_integar(command_decoded_data);
                default:
                    is_decoder_command_error <= 1;
            endcase
        else
        begin
            do_start_generator <= 0;
            do_stop_generator <= 0;
            do_start_status_transmitting <= 0;
            is_decoder_command_error <= 0;
        end
    end
    
    
    // if reset/waveform parameters changed, do_start_wave_calculator
    assign voltage1_changed = (voltage1 != voltage1_in);
    assign voltage2_changed = (voltage2 != voltage2_in);
    assign polar_changed = (polar != polar_in);
    assign settling_divisor_changed = (settling_divisor != settling_divisor_in);
    assign delay_divisor_changed = (delay_divisor != delay_divisor_in);
    assign wave_points_changed = (wave_points != wave_points);
    assign parameters_changed = voltage1_changed || voltage2_changed || polar_changed || settling_divisor_changed || delay_divisor_changed || wave_points_changed;
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
        begin
            voltage1 <= 32768;
            voltage2 <= 32768;
            polar <= 0;
            settling_divisor <= 10;
            delay_divisor <= 10;
            wave_points <= 2048;
        end
        else if (parameters_changed && !generator_is_running && !wave_calculator_is_running)
        begin
            voltage1 <= voltage1_in;
            voltage2 <= voltage2_in;
            polar <= polar_in;
            settling_divisor <= settling_divisor_in;
            delay_divisor <= delay_divisor_in;
            wave_points <= wave_points_in;
        end
    end
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            wave_ready <= 0;
        else if (do_start_wave_calculator)
            wave_ready <= 1;
    end
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            do_start_wave_calculator <= 0;
        else if (do_start_wave_calculator)
            do_start_wave_calculator <= 0;
        else if (!wave_ready)
            do_start_wave_calculator <= 1;
        else if (parameters_changed && !generator_is_running && !wave_calculator_is_running)
            do_start_wave_calculator <= 1;
    end
    
    
    // change steps and down_sampling_divisor
    assign steps_changed = (steps != steps_in);
    assign down_sampling_divisor_changed = (down_sampling_divisor != down_sampling_divisor_in);
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
        begin
            steps <= 1;
            down_sampling_divisor <= 10;
        end
        else if ((steps_changed || down_sampling_divisor_changed) && !generator_is_running)
        begin
            steps <= steps_in;
            down_sampling_divisor <= down_sampling_divisor_in;
        end
    end
    
    
    // command encoding
    assign command_encoded = 
    {
        ":", "GNRT", encoder_integar(generator_is_running), ";",
        ":", "WVCL", encoder_integar(wave_calculator_is_running), ";",
        ":", "ERRO", encoder_integar(error_flag), ";",
        ":", "VLT1", encoder_integar(voltage1), ";",
        ":", "VLT2", encoder_integar(voltage2), ";",
        ":", "POLA", encoder_integar(polar), ";",
        ":", "STTM", encoder_integar(settling_divisor), ";",
        ":", "DLTM", encoder_integar(delay_divisor), ";",
        ":", "PNTS", encoder_integar(wave_points), ";",
        ":", "STEP", encoder_integar(steps), ";",
        ":", "SAMP", encoder_integar(down_sampling_divisor), ";"
    };
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            command_encoded_reg <= 0;
        else if (do_start_status_transmitting)
            command_encoded_reg <= command_encoded;
    end
    
    
    // status_transmitting_in_process
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            status_transmitting_in_process <= 0;
        else if (do_start_status_transmitting)
            status_transmitting_in_process <= 1;
        else if (!encoder_counter_is_running && tx_shift_empty && tx_ready)
            status_transmitting_in_process <= 0;
    end
    
    
    // encoder_counter
    assign encoder_counter_is_zero = (encoder_counter == 0);
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            encoder_counter_is_running <= 0;
        else if (do_start_status_transmitting)
            encoder_counter_is_running <= 1;
        else if (encoder_counter_is_zero && do_send_char)
            encoder_counter_is_running <= 0;
    end
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            encoder_counter <= 8'd109;
        else if (encoder_counter_is_zero && do_send_char)
            encoder_counter <= 8'd109;
        else if (encoder_counter_is_running && do_send_char)
            encoder_counter <= encoder_counter - 1'd1;
    end
    
    
    // transmitting encoder
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            do_send_char <= 0;
        else if (do_send_char)
            do_send_char <= 0;
        else if (encoder_counter_is_running && tx_shift_empty && tx_ready)
            do_send_char <= 1;
    end
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            tx_data <= 0;
        else if (encoder_counter_is_running && tx_shift_empty && tx_ready)
        begin
            tx_data[7] <= command_encoded_reg[encoder_counter * 8 + 7];
            tx_data[6] <= command_encoded_reg[encoder_counter * 8 + 6];
            tx_data[5] <= command_encoded_reg[encoder_counter * 8 + 5];
            tx_data[4] <= command_encoded_reg[encoder_counter * 8 + 4];
            tx_data[3] <= command_encoded_reg[encoder_counter * 8 + 3];
            tx_data[2] <= command_encoded_reg[encoder_counter * 8 + 2];
            tx_data[1] <= command_encoded_reg[encoder_counter * 8 + 1];
            tx_data[0] <= command_encoded_reg[encoder_counter * 8 + 0];
        end
    end
    
    
    // error_flag
    assign error_flag = {decoder_framing_error, UART_error_flag};
    assign clear_error_flag = do_start_status_transmitting;
    
    
    // handle UART_error_flag
    assign clear_UART_error_flag = clear_error_flag;
    
    
    // handle decoder_framing_error
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            decoder_framing_error <= 0;
        else if (is_decoder_command_error || is_decoder_key_word_error)
            decoder_framing_error <= 1;
        else if (clear_error_flag)
            decoder_framing_error <= 0;
    end
    
    
    // uart module
    uart uart_0(
    
    
        // inputs:
        .clk(clk),
        .rst_n(rst_n),
        .rxd(rxd),
        .baud_divisor(baud_divisor),
        .do_read_char(do_read_char),
        .tx_data(tx_data),
        .do_force_break(do_force_break),
        .do_send_char(do_send_char),
        .clear_UART_error_flag(clear_UART_error_flag),
        
        
        // outputs:
        .rx_data(rx_data),
        .rx_char_ready(rx_char_ready),
        .break_detect(break_detect),
        .txd(txd),
        .tx_ready(tx_ready),
        .tx_shift_empty(tx_shift_empty),
        .UART_error_flag(UART_error_flag)
    
    
    );
    
    
endmodule
