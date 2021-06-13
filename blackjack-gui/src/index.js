import React from 'react';
// import Text from 'react-native';
import ReactDOM from 'react-dom';
import './index.css';
// import './index.html';
import ROSLIB from "roslib"
import { Button } from "./components/Button";
import TextField from '@material-ui/core/TextField'


// 'use strict';
var ros = new ROSLIB.Ros({
url : 'ws://192.168.1.129:9090' //10.19.99.150 ws://10.16.204.59 192.168.1.111
});

ros.on('connection', function() {
console.log('Connected to server.');
});

ros.on('error', function(error) {
  console.log('Error.');
});

ros.on('close', function() {
  console.log('Closed.');
});

var txt_listener = new ROSLIB.Topic({
ros : ros,
name : '/txt_msg',
messageType : 'std_msgs/String'
});

txt_listener.subscribe(function(m) {
  console.log('Received message :' + JSON.stringify(m));
});

// var txt_publisher = new ROSLIB.Topic({
//   ros : ros,
//   name : '/get_msg',
//   messageType : 'std_msgs/String'
//   });

var deps_publisher = new ROSLIB.Topic({
ros : ros,
name : '/player_join',
messageType : 'std_msgs/String'
});

var bets_publisher = new ROSLIB.Topic({
  ros : ros,
  name : '/player_bets',
  messageType : 'std_msgs/String'
  });

var htst_publisher = new ROSLIB.Topic({
  ros : ros,
  name : '/player_hit_stay',
  messageType : 'std_msgs/String'
  });

// var str = new ROSLIB.Message({
//   data : 'Player joined.'
// });

// var seat1_msg = new ROSLIB.Message({
//   data : 'Seat 1 selected.'
// });

// var seat2_msg = new ROSLIB.Message({
//   data : 'Seat 2 selected.'
// });

// var seat3_msg = new ROSLIB.Message({
//   data : 'Seat 3 selected.'
// });

var hit_msg = null;

var stay_msg = null;

// var serv_requested_join = false
// var serv_requested_seats = false
// var serv_requested_deps = false
// var serv_requested_bets = false
// var serv_requested_htst = false;

var game_status = new ROSLIB.Topic({
  ros : ros,
  name : '/game_status',
  messageType : 'std_msgs/Int64'
  });

var balance = null;

game_status.subscribe(function(message) {
  console.log('Received message :' + JSON.stringify(message.data));
  balance = message.data
  // if (message.data === 40) {
  //   serv_requested_join = true;
  // }
  // if (message.data === 50) {
  //   serv_requested_seats = true;
  // }
  // if (message.data === 60) {
  //   serv_requested_deps = true;
  // }
  // if (message.data === 70) {
  //   serv_requested_bets = true;
  // }
  // if (message.data === 80) {
  //   serv_requested_htst = true;
  // }
});

var deps_msg = null;
var bets_msg = null;

class Game extends React.Component {
    // Set up props
  constructor(props) {
    super(props);
    this.state = {
      joined: false,
      seat1: false,
      seat2: false,
      seat3: false,
      joinFlag: false,
      seatFlag: false,
      seatsVisible: false,
      deposit: null,
      bet: null,
      round_start: false,
      hit_stay_done: false,
      game_end: false,
      hit: false,
      stay: false,
    };
  }

  handleClick() {
    // if (serv_requested_join) {
    // txt_publisher.publish(str);
    this.setState({
      joined: true,
    });
    // }
  }

  handleClickSeat() {
    // if (serv_requested_seats) {
    // txt_publisher.publish(seat1_msg);
    this.setState({
      seat1: true,
      seatFlag: true
    });
    // }
  }

  handleClickSeat2() {
    // if (serv_requested_seats) {
    // txt_publisher.publish(seat2_msg);
    this.setState({
      seat2: true,
      seatFlag: true
    });
    // }
  }

  handleClickSeat3() {
    // if (serv_requested_seats) {
    // txt_publisher.publish(seat3_msg);
    this.setState({
      seat3: true,
      seatFlag: true
    });
    // }
  }

  handleDeps(e) {
    var seatID = null;

    if ((e.key === 'Enter') /*&& (serv_requested_deps)*/) {
      console.log('Enter key pressed')
      
      if (this.state.seat1) {
        seatID = '0,'
      }
      else if (this.state.seat2) {
        seatID = '1,'
      }
      else {
        seatID = '2,'
      }

      deps_msg = new ROSLIB.Message({
      data : seatID + e.target.value
      });

      deps_publisher.publish(deps_msg);
      this.setState({deposit: e.target.value})
    }
  }

  handleBets(e) {
    var seatID = null;

    if ((e.key === 'Enter') /* && (serv_requested_bets)*/) {
      console.log('Enter key pressed')

      if (this.state.seat1) {
        seatID = '0,'
      }
      else if (this.state.seat2) {
        seatID = '1,'
      }
      else {
        seatID = '2,'
      }

      bets_msg = new ROSLIB.Message({
      data : seatID + e.target.value
      });

      bets_publisher.publish(bets_msg);
      this.setState({bet: e.target.value})

      // serv_requested_bets = false;
    }
  }

  handleClickHit() {
    var seatID = null;

    // if (serv_requested_htst) {
      
      if (this.state.seat1) {
        seatID = '0,'
      }
      else if (this.state.seat2) {
        seatID = '1,'
      }
      else {
        seatID = '2,'
      }

      hit_msg = new ROSLIB.Message({
        data : seatID + 'hit'
      });

      htst_publisher.publish(hit_msg);

      this.setState({
        hit: true,
      });
    // }
  }

  handleClickStay() {
    var seatID = null;

    // if (serv_requested_htst) {
    
      if (this.state.seat1) {
        seatID = '0,'
      }
      else if (this.state.seat2) {
        seatID = '1,'
      }
      else {
        seatID = '2,'
      }

      stay_msg = new ROSLIB.Message({
        data : seatID + 'stay'
      });
      htst_publisher.publish(stay_msg);
      this.setState({
        stay: true,
      });
    // }
  }


  render() {

    return (
      <div className="game">
        {/* <div className="game-board">
          <Board />
        </div> */}
        <div>
          {/* The joiin button becomes visible when a request is sent from the server*/}
          {/* { serv_requested ? <Button onClick={() => {
            this.setState({ joined: true });
          }}>Join Game</Button>: null} */}
        {/* The join button remains visible untill it is pressed. */}
          {this.state.joined ? null : <Button onClick={() =>
              this.handleClick()
            }>Join Game</Button>}
            {/* The seat 1 button becomes visible when this.state.joined is true and the seat button has not been pressed */}
          {!(this.state.seat1) && this.state.joined && !(this.state.seatFlag) ?  <Button onClick={() =>
              this.handleClickSeat()
            }>Seat 1</Button>   : null}
          {!(this.state.seat2) && this.state.joined && !(this.state.seatFlag)  ?  <Button onClick={() => 
              this.handleClickSeat2()
            }>Seat 2</Button>   : null}
          {!(this.state.seat3) && this.state.joined && !(this.state.seatFlag) ?  <Button onClick={() =>
              this.handleClickSeat3()
            }>Seat 3</Button>   : null}
            {(this.state.seatFlag) && (this.state.deposit == null) ?
            <TextField
              label="Place Deposit"
              onKeyPress= {(e) => this.handleDeps(e)
             }
              type="number"
              helperText="Please place a deposit for the game (AUD)."
              defaultValue="50"
              >
              </TextField> : null
            }
            {(this.state.deposit != null) ?
            <TextField
              label="Place Bet"
              onKeyPress= {(e) => this.handleBets(e)
             }
              type="number"
              helperText="Please place a bet for the game (AUD). Your bet must be lower than the deposit entered."
              defaultValue="10"
              >
              </TextField> : null
            }
            {!(this.state.hit_stay_done) && (this.state.bet != null) ?  <Button onClick={() =>
              this.handleClickHit()
            }>Hit</Button>   : null}
            {!(this.state.hit_stay_done) && (this.state.bet != null) ?  <Button onClick={() =>
              this.handleClickStay()
            }>Stay</Button>   : null}
             {/* {!(this.state.hit_stay_done) && (this.state.bet != null) ? 
              <TextField
              label="Current Balance"
              // onKeyPress= {(e) => this.handleBets(e)}
              type="number"
              value={balance}
              // helperText="Please place a bet for the game (AUD). Your bet must be lower than the deposit entered."
              // defaultValue="10"
              >
              </TextField> : null} */}


        </div>
        <div className="game-info">
          <div>{/* status */}</div>
          <ol>{/* TODO */}</ol>
        </div>
      </div>
    );
  }
}

// ========================================

ReactDOM.render(
  <Game />,
  document.getElementById('root')
);

