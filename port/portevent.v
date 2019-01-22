/**
 * @file portevent.v
 * @breif port event state machine for Modbus protocal.
 * @auther denghb
 * @date 20190119
 * @version 1.0
 * @license GNU General Public License (GPL) 3.0 
 *
 *_______________________________________________________________________
 * History:
 *  <Date>      |   <version>   |     <Author>      |     <Description>
 *  20190119    |   1.0	        |     denghb        |     first created
 *_______________________________________________________________________
**/


module portevent(
    // inputs
    clk,
    rst_n,
    
    
    
    // inouts
    
    
    // outputs
    outEventInQueue,
    outEventHappened
);

    wire MBPortEventPost;
    wire MBPortEventGet;

    /**
     * @brief MB protocal Event State Machine
    **/
    always @ ( posedge clk or negedge rst_n )
    begin
        if ( !rst_n )
            regEventInQueue <= FALSE;
        else if ( MBPortEventPost )
            regEventInQueue <= TRUE;
        else if ( MBPortEventGet )
            regEventInQueue <= FALSE;
    end
    always @ ( posedge clk or negedge rst_n )
    begin
        if ( !rst_n )
            regQueuedEvent <= EV_READY;
        else if ( MBPortEventPost )
            regQueuedEvent <= eEvent;
    end
    
endmodule