// web_page.h

#ifndef _WEB_PAGE_h
#define _WEB_PAGE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
char webpage[] PROGMEM = R"=====(
<!DOCTYPE HTML><html>
<!--DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN" "http://www.w3.org/TR/html1/DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="cz" lang="cz">
<script src='https://kit.fontawesome.com/a076d05399.js'></script-->
x
<head>
  <title>wifi control</title>
  <meta charset="utf-8" http-equiv="connection:Keep-Alive">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="content-type" content="text/html;charset=utf-8" />
  <meta name="generator" content="Geany 1.30.1" />
  <link rel="icon" href="data:;base64,=">
  <!--link rel="stylesheet" href="https://stackpath.bootstrapcdn.com/bootstrap/4.4.1/css/bootstrap.min.css"
    integrity="sha384-Vkoo8x4CGsO3+Hhxv8T/Q5PaXtkKtu6ug5TOeNV6gBiFeWPGFN9MuhOf23Q9Ifjh" crossorigin="anonymous"-->
</head>
<style>
  /* 
  ##Device = Desktops
  ##Screen = 1281px to higher resolution desktops
*/

  @media (min-width: 1281px) {

    body,
    html {
      text-align: center;
    }

    .btn {
      width: 105px;
      height: 100px;
    }

    th,
    input,
    optgroup,
    select,
    textarea {
      font-size: 40px;
    }

    .main {
      height: 700px;
      width: 800px;

    }

    .space {
      height: 75px;
    }

    .mainTable {
      margin: auto;
    }
  }

  button[type=submit] {
    margin-top: 30px;
    font-size: 30px;
  }

  input[type=radio]+label {
    font-size: 35px;
  }

  input[type=number] {
    width: 140px;
  }

  #tempLabel {
    font-size: 39px;
  }

  /* 
  ##Device = Laptops, Desktops
  ##Screen = B/w 1025px to 1280px
*/

  @media (min-width: 1025px) and (max-width: 1280px) {

    body,
    html {
      text-align: center;
    }

    .main {
      width: 600px;

    }

    .mainTable {
      margin: auto;
    }

    .p {
      text-align: center;
    }

    .space {
      height: 200px;
    }

    .btn {
      width: 135px;
      height: 80px;
    }


    .setTemp {
      height: 50px;
      width: 100px;
      font-size: 34px;

    }

    .setTime {
      height: 49px;
      width: 130px;
      font-size: 34px;
    }

    th,
    input,
    optgroup,
    select,
    textarea {
      font-size: 36px;
    }

    button[type=submit] {
      margin-top: 150px;
      font-size: 30px;
    }

    .adcLabel,
    tempLabel {  
    font-size: 43px;
    }

    .tmrLabel {
      font-size: 50px;
    }
  }

  /* 
  ##Device = Tablets, Ipads (portrait)
  ##Screen = B/w 768px to 1024px
*/

  @media (min-width: 768px) and (max-width: 1024px) {


    body,
    html {
      text-align: center;
    }

    .btn {
      width: 120px;
      height: 100px;
    }

  }

  /* 
  ##Device = Tablets, Ipads (landscape)
  ##Screen = B/w 768px to 1024px
*/

  @media (min-width: 768px) and (max-width: 1024px) and (orientation: landscape) {

    .btn {
      width: 105px;
      height: 100px;
    }

  }




  /* 
  ##Device = Low Resolution Tablets, Mobiles (Landscape)
  ##Screen = B/w 481px to 767px
*/

  @media (min-width: 481px) and (max-width: 767px) {

    body,
    html {
      height: 100%;
      margin: 0;
      text-align: center;
    }

    .btn {
      width: 120px;
      height: 130px;
    }


    .main {
      width: 100%;
      height: 100%;

    }

    .mainTable {
      margin: auto;
    }

    .space {
      height: 150px;
    }

    th,
    button,
    input,
    optgroup,
    select,
    textarea {
      font-size: 56px;
    }

    .tablink {
      font-size: 30px;
    }

    input[type=radio]+label {
      font-size: 35px;
    }

    input[type=time],
    input[type=number] {
      font-size: 60px;
      width: auto;
    }

    button[type=submit] {
      margin-top: 150px;
      font-size: 30px;
    }

    .setTemp {
      width: 140px;
      text-align: center;
    }

    .spc {
      width: 20px;
    }

    p {
      font-size: 25px;
    }

    .tmrLabel {
      font-size: 50px;
    }
  }

  /* 
  ##Device = Most of the Smartphones Mobiles (Portrait)
  ##Screen = B/w 320px to 479px
*/

  @media (min-width: 320px) and (max-width: 480px) {

    body,
    html {
      height: 100%;
      margin: 0;
      text-align: center;
    }

    .tmrLabel {
      font-size: 50px;
    }

    .btn {
      width: 60px;
      height: 60px;
      font-size: 15px;
    }

    input[type=radio]+label,
    input[type=checkbox]+label {
      font-size: 18px;
    }

    input[type=time],
    input[type=number] {
      font-size: 30px;
      width: auto;
    }

    button[type=submit] {
      margin-top: 30px;
      font-size: 25px;
    }
  }

  /*radio buttons*/
  input[type=radio],
  input[type=checkbox] {
    display: none;
  }

  input[type=radio]+label,
  input[type=checkbox]+label {
    display: inline-block;
    margin: -2px;
    padding: 4px 12px;
    margin-bottom: 0;
    /*font-size: 23px;*/
    line-height: 40px;
    color: #333;
    text-align: center;
    text-shadow: 0 1px 1px rgba(255, 255, 255, 0.75);
    vertical-align: middle;
    cursor: pointer;
    background-color: #f5f5f5;
    background-image: -moz-linear-gradient(top, #fff, #e6e6e6);
    background-image: -webkit-gradient(linear, 0 0, 0 100%, from(#fff), to(#e6e6e6));
    background-image: -webkit-linear-gradient(top, #fff, #e6e6e6);
    background-image: -o-linear-gradient(top, #fff, #e6e6e6);
    background-image: linear-gradient(to bottom, #fff, #e6e6e6);
    background-repeat: repeat-x;
    border: 1px solid #ccc;
    border-color: #e6e6e6 #e6e6e6 #bfbfbf;
    border-color: rgba(0, 0, 0, 0.1) rgba(0, 0, 0, 0.1) rgba(0, 0, 0, 0.25);
    border-bottom-color: #b3b3b3;
    filter: progid: DXImageTransform.Microsoft.gradient(startColorstr='#ffffffff', endColorstr='#ffe6e6e6', GradientType=0) filter: progid: DXImageTransform.Microsoft.gradient(enabled=false);
    -webkit-box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.2), 0 1px 2px rgba(0, 0, 0, 0.05);
    -moz-box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.2), 0 1px 2px rgba(0, 0, 0, 0.05);
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.2), 0 1px 2px rgba(0, 0, 0, 0.05);
  }

  input[type=radio]:checked+label,
  input[type=checkbox]:checked+label {
    background-image: none;
    outline: 0;
    -webkit-box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.15), 0 1px 2px rgba(0, 0, 0, 0.05);
    -moz-box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.15), 0 1px 2px rgba(0, 0, 0, 0.05);
    box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.15), 0 1px 2px rgba(0, 0, 0, 0.05);
    background-color: lightgreen;
  }

  .btn {
    background-color: gray;
  }

  /* Style tab links */

  .tablink {
    background-color: #555;
    color: white;
    float: left;
    border: none;
    outline: none;
    cursor: pointer;
    padding: 14px 16px;
    /*font-size: 17px;*/
    width: 25%;
  }

  .tablink:hover {
    background-color: #777;
  }

  /* Style the tab content (and add height:100% for full page content) */

  .tabcontent {
    color: white;
    display: none;
    padding: 100px 20px;
    height: 100%;
    font-size: 15px;

  }


  /*.p{
  text-align: center;
}*/

  #termostat {
    background-color: #36bd36;
    font-size: 15px;
  }

  #prehled {
    background-color: #3f3ffd;
  }

  #spinacky {
    background-color: #e6be5a;
  }

  #nastaveni {
    background-color: #f6534ef6;
  }

  /*-----------------------------------------*/
</style>

<script>
  var gateway = 'ws://' + window.location.hostname + ':81/';
  var Socket;
  window.addEventListener('load', onLoad);
//  function initWebSocket() {
//    console.log('Trying to open a WebSocket connection...');
//    Socket = new WebSocket(gateway);
//    Socket.onopen    = onOpen;
//    Socket.onclose   = onClose;
//    Socket.onmessage = onMessage; // <-- add this line
//  }

  var termostatLabel, spinackyLabel, setIOlabel;
  function init() {
    console.log('Trying to open a WebSocket connection...');
    Socket = new WebSocket(((window.location.protocol === "https:") ? "wss://" : "ws://") + window.location.host + "/ws");
    //Socket = new WebSocket(gateway);
    Socket.onmessage =  function onOpen(event) {console.log('Connection opened');}
    Socket.onclose =  function onClose(event) {console.log('Connection closed');}

    Socket.onmessage = function (event) {
      if(event.data[1]=='I'){
        switch(event.data[2]){
          case "1": if(event.data[4]=='n')document.getElementById("in1").style.backgroundColor = "goldenrod";
               else document.getElementById("in1").style.backgroundColor = "gray";
               break;
          case "2": if(event.data[4]=='n')document.getElementById("in2").style.backgroundColor = "goldenrod";
               else document.getElementById("in2").style.backgroundColor = "gray";
               break;   
          case "3": if(event.data[4]=='n')document.getElementById("in3").style.backgroundColor = "goldenrod";
               else document.getElementById("in3").style.backgroundColor = "gray";
               break;  
          case "4": if(event.data[4]=='n')document.getElementById("in4").style.backgroundColor = "goldenrod";
               else document.getElementById("in4").style.backgroundColor = "gray";
               break;                         
        }
      }
    if(event.data[1]=='O'){
        switch(event.data[2]){
          case "1": if(event.data[4]=='n')document.getElementById("out1").style.backgroundColor = "green";
               else document.getElementById("out1").style.backgroundColor = "gray";
               break;
          case "2": if(event.data[4]=='n')document.getElementById("out2").style.backgroundColor = "green";
               else document.getElementById("out2").style.backgroundColor = "gray";
               break;   
          case "3": if(event.data[4]=='n')document.getElementById("out3").style.backgroundColor = "green";
               else document.getElementById("out3").style.backgroundColor = "gray";
               break;  
          case "4": if(event.data[4]=='n')document.getElementById("out4").style.backgroundColor = "green";
               else document.getElementById("out4").style.backgroundColor = "gray";
               break;           
          case "5": if(event.data[4]=='n')document.getElementById("out5").style.backgroundColor = "green";
               else document.getElementById("out5").style.backgroundColor = "gray";
               break;  
          case "6": if(event.data[4]=='n')document.getElementById("out6").style.backgroundColor = "green";
               else document.getElementById("out6").style.backgroundColor = "gray";
               break;              

        }
      }

    if(event.data[1]=='A'){
      a_val = event.data.substring(2);
      document.getElementById("adcValLabel").innerHTML = a_val;
    }
    if(event.data[1]=='T'){
      var t_val = event.data.substring(2);
      document.getElementById("tempLabel1").innerHTML = t_val;
       document.getElementById("tempLabel1_1").innerHTML = t_val;
    }
    if(event.data[1]=='#'){
      SortData(event.data);
    }
    }
    document.getElementById("defaultOpen").click();
  }

 function getData(){
  Socket.send("&S");
 }

  function onLoad(event) {
    init();
  }

  function SortData(msg) {
    //rozsekat zpravu
//&#T*0_0*0_0*0_0*0_0*#S*0_0*0_0*0_0*0_0*#O*0*0*#IO*0_0|0*0_1|0*0_2|0
    var recTstat = msg.substring(msg.indexOf('T') + 3, msg.indexOf('S'));
    var recTstatPole = recTstat.split('*');
    var recInp = msg.substring(msg.indexOf('I') + 2, msg.indexOf('O'));

    //stav vystupu
    var recOutStat = msg.substring(msg.indexOf("OST") + 3, msg.indexOf("END"));

    // if(recOutStat.substring(1,2) == "1")document.getElementById("out1").style.backgroundColor = "green";
    // else document.getElementById("out1").style.backgroundColor = "gray";    

    // if(recOutStat.substring(2,3)== "1")document.getElementById("out2").style.backgroundColor = "green";
    // else document.getElementById("out2").style.backgroundColor = "gray";
    // if(recOutStat.substring(3,4)== "1")document.getElementById("out3").style.backgroundColor = "green";
    // else document.getElementById("out3").style.backgroundColor = "gray";    
    // if(recOutStat.substring(4,5)== "1")document.getElementById("out4").style.backgroundColor = "green";
    // else document.getElementById("out4").style.backgroundColor = "gray";
    // if(recOutStat.substring(5,6)== "1")document.getElementById("out5").style.backgroundColor = "green";
    // else document.getElementById("out5").style.backgroundColor = "gray";    
    // if(recOutStat.substring(6,7)== "1")document.getElementById("out6").style.backgroundColor = "green";
    // else document.getElementById("out5").style.backgroundColor = "gray"                 
    //stav vstupu
     var recInStat = msg.substring(msg.indexOf("IST") + 3, msg.indexOf("OST")-1);
    if(recInStat.substring(1,2) == "1")document.getElementById("in1").style.backgroundColor = "goldenrod";
    else document.getElementById("in1").style.backgroundColor = "gray";    
    if(recInStat.substring(2,3)== "1")document.getElementById("in2").style.backgroundColor = "goldenrod";
    else document.getElementById("in2").style.backgroundColor = "gray";
    if(recInStat.substring(3,4)== "1")document.getElementById("in3").style.backgroundColor = "goldenrod";
    else document.getElementById("in3").style.backgroundColor = "gray";    
    if(recInStat.substring(4,5)== "1")document.getElementById("in4").style.backgroundColor = "goldenrod";
    else document.getElementById("in4").style.backgroundColor = "gray";


    //termostat
    var teploty = document.getElementsByClassName("setTemp");
    var casy = document.getElementsByClassName("setTime");
    for (i = 0; i < teploty.length; i++) {
      var tp = recTstatPole[i].substr( recTstatPole[i].indexOf('_')+1);
      var fl = parseFloat(tp.substring(0,2)+'.'+tp.substring(2));
      teploty[i].value = fl;
      casy[i].valueAsNumber = parseInt( recTstatPole[i].substr(0,recTstatPole[i].indexOf('_')))*60000;;
    }

    //spinacky
    var casyOn = document.getElementsByClassName("setTimeSpinOn");
    var casyOff = document.getElementsByClassName("setTimeSpinOff");
    var recSpin = msg.substring(msg.indexOf('S') + 2, msg.indexOf('O')); 
    var recSpinPole = recSpin.split('*');
    for (i = 0; i < casyOn.length; i++) {
      casyOn[i].valueAsNumber = parseInt( recSpinPole[i].substr(0,recSpinPole[i].indexOf('_')))*60000;
      casyOff[i].valueAsNumber = parseInt(recSpinPole[i].substr(recSpinPole[i].indexOf('_') + 1))*60000;
    }

    //vystupy termostat,spinacky
    var outs = msg.substring(msg.indexOf('O') + 2, msg.indexOf('I')); 
    var prepinacTstat = document.getElementsByName("prepTstat");
    var prepinacSpina = document.getElementsByName("prepSpin");
    var outsPole = outs.split('*');
    var out_tstat = parseInt(outsPole[0]);
    for (i = 0; i < prepinacTstat.length; i++) {
      if(out_tstat == i){
        prepinacTstat[i].checked = true;
        OutputSelect(prepinacTstat[i]);
      }
    }
    var out_spina = parseInt(outsPole[1]);
    for (i = 0; i < prepinacSpina.length; i++) {
      if(out_spina == i){
      prepinacSpina[i].checked = true;  
      OutputSelect(prepinacSpina[i]);
      }
    }
    //nastaveni IO
    var IOs = msg.substring(msg.indexOf('I') + 3);
    var IOsPole = IOs.split('*');
    var prep_mode1 = document.getElementsByName("prepInMode1");
    var prep_mode2 = document.getElementsByName("prepInMode2");
    var prep_mode3 = document.getElementsByName("prepInMode3");
    var prep_mode4 = document.getElementsByName("prepInMode4");
    var prepsInp1_ = document.getElementsByName("prepInp1");
    var prepsInp2_ = document.getElementsByName("prepInp2");
    var prepsInp3_ = document.getElementsByName("prepInp3");
    var prepsInp4_ = document.getElementsByName("prepInp4");
    var timers = document.getElementsByClassName("timer");
    for ( ii = 0; ii < IOsPole.length; ii++) {
      var mode_ = parseInt(IOsPole[ii].substring(0,IOsPole[ii].indexOf('_')));
      var out_ = parseInt(IOsPole[ii].substring(IOsPole[ii].indexOf('_')+1,IOsPole[ii].indexOf('|')));
      var time_ =parseInt(IOsPole[ii].substring(IOsPole[ii].indexOf('|')+1));
      for( j = 0; j < 3; j++){
        if(ii == 0){if(mode_ == j){prep_mode1[j].checked = true;OutputSelect(prep_mode1[j]);}}
        if(ii == 1){if(mode_ == j){prep_mode2[j].checked = true;OutputSelect(prep_mode2[j]);}}
        if(ii == 2){if(mode_ == j){prep_mode3[j].checked = true;OutputSelect(prep_mode3[j]);}}
        if(ii == 3){if(mode_ == j){prep_mode4[j].checked = true;OutputSelect(prep_mode4[j]);}}
      }
      if(out_ != -1){
      if(ii == 0){prepsInp1_[out_].checked = true;OutputSelect(prepsInp1_[out_])}
      if(ii == 1){prepsInp2_[out_].checked = true;OutputSelect(prepsInp2_[out_])}
      if(ii == 2){prepsInp3_[out_].checked = true;OutputSelect(prepsInp3_[out_])}
      if(ii == 3){prepsInp4_[out_].checked = true;OutputSelect(prepsInp4_[out_])}
     
      }
      timers[ii].valueAsNumber = parseInt(time_)*1000; 
    }

  }

  function SaveData() {
    var descripts = ["relé 1", "relé 2", "out 1", "out 2"];
    var message = "&#T*";//zprava k odeslani
    var radioTstat = document.getElementsByName("prepTstat");
    var radioSpin = document.getElementsByName("prepSpin");
    var labels = document.getElementsByClassName("label");


    //odeslani zpravy
    //termostat: teplota format 253(25,3) cas v minutach

    var teploty = document.getElementsByClassName("setTemp");
    var casy = document.getElementsByClassName("setTime");
    for (i = 0; i < teploty.length; i++) {
      var num = teploty[i].valueAsNumber * 10;
      message += num.toString() + '_';
      message += ToMinutes(casy[i].valueAsNumber) + "*";
    }
    var spinaciCasyOn = document.getElementsByClassName("setTimeSpinOn");
    var spinaciCasyOff = document.getElementsByClassName("setTimeSpinOff");
    message += "#S*"
    for (i = 0; i < spinaciCasyOn.length; i++) {

      message += ToMinutes(spinaciCasyOn[i].valueAsNumber) + "_";
      message += ToMinutes(spinaciCasyOff[i].valueAsNumber) + "*";
    }
    const outindex = Array.from(document.getElementsByName("prepTstat"));
    const idx = outindex.findIndex(x => x.checked == true);
    message += "#O*" + idx.toString();
    const outindex1 = Array.from(document.getElementsByName("prepSpin"));
    const idx1 = outindex1.findIndex(x => x.checked == true);
    message += "*" + idx1.toString();

    //data IO
    var IOtimers = document.getElementsByClassName("timer");
    message += "*#IO" 
    //IO 1
    const indexMode1 = Array.from(document.getElementsByName("prepInMode1"));
    const idxMode1 = indexMode1.findIndex(x => x.checked == true);
    message += "*" + idxMode1.toString() + "_";
    const outindex2 = Array.from(document.getElementsByName("prepInp1"));
    const idx2 = outindex2.findIndex(x => x.checked == true);
    message += idx2.toString()+ "|";
    message += ToSeconds(IOtimers[0].value) + "*";
    //IO 2
    const indexMode2 = Array.from(document.getElementsByName("prepInMode2"));
    const idxMode2 = indexMode2.findIndex(x => x.checked == true);
    message += idxMode2.toString() + "_";
    const outindex3 = Array.from(document.getElementsByName("prepInp2"));
    const idx3 = outindex3.findIndex(x => x.checked == true);
    message +=  idx3.toString()+ "|";
    message += ToSeconds(IOtimers[1].value) + "*";
    //IO 3
    const indexMode3 = Array.from(document.getElementsByName("prepInMode3"));
    const idxMode3 = indexMode3.findIndex(x => x.checked == true);
    message += idxMode3.toString() + "_";
    const outindex4 = Array.from(document.getElementsByName("prepInp3"));
    const idx4 = outindex4.findIndex(x => x.checked == true);
    message += idx4.toString()+ "|";
    message += ToSeconds(IOtimers[2].value) + "*";
    //IO 4
    const indexMode4 = Array.from(document.getElementsByName("prepInMode4"));
    const idxMode4 = indexMode4.findIndex(x => x.checked == true);
    message +=  idxMode4.toString() + "_";
    const outindex5 = Array.from(document.getElementsByName("prepInp4"));
    const idx5 = outindex5.findIndex(x => x.checked == true);
    message +=  idx5.toString()+ "|";
    message += ToSeconds(IOtimers[3].value) + "*";
    Socket.send(message);
  }

  //funkce pro prevod casu na minuty
  function ToMinutes(cas) {  
    var pom = parseInt(cas)/60000;
    return pom.toString();
  }

      //funkce pro prevod casu na minuty
      function ToSeconds(cas) {
        if(cas.length<6)cas+=":00"
        var pom = parseInt(cas.substr(0, 2));
        var pom2 = parseInt(cas.substr(3, 2));
        var pom3 = parseInt(cas.substr(6, 2));
        return ((pom * 3600) + (pom2*60) + pom3);
      }
  //funkce pro prevod minut na HH:MM
  function FromMinutes(cas) {
    var pom = parseInt(cas);
    var pom1 = ~~(pom / 60);
    var pom2 = pom % 60;

    return pom1.toString().padStart(2,'0') + ':' + pom2.toString().padStart(2,'0');
  }

    //funkce pro prevod sekund na HH:MM:SS
    function FromSeconds(cas) {
    var pom = parseInt(cas);
    var pom1 = ~~(pom / 3600);
    var pom2 = ~~(pom / 60) - pom1 * 60;
    var pom3 = pom % 60;

    return pom1.toString().padStart(2,'0') + ':' + pom2.toString().padStart(2,'0')+ ':' + pom3.toString().padStart(2,'0');
  }
  function btn1click() {
    var btn = document.getElementById("out1");
    if (btn.style.backgroundColor == "") btn.style.backgroundColor = "gray";   
    if (btn.style.backgroundColor == "gray")Socket.send("#C*1on");       
    else Socket.send("#C*1of");
  }
  function btn2click() {
    var btn = document.getElementById("out2");
    if (btn.style.backgroundColor == "") btn.style.backgroundColor = "gray";   
    if (btn.style.backgroundColor == "gray")Socket.send("#C*2on");       
    else Socket.send("#C*2of");
  }
  function btn3click() {
    var btn = document.getElementById("out3");
    if (btn.style.backgroundColor == "") btn.style.backgroundColor = "gray";   
    if (btn.style.backgroundColor == "gray")Socket.send("#C*3on");       
    else Socket.send("#C*3of");
  }
  function btn4click() {
    var btn = document.getElementById("out4");
    if (btn.style.backgroundColor == "") btn.style.backgroundColor = "gray";   
    if (btn.style.backgroundColor == "gray")Socket.send("#C*4on");       
    else Socket.send("#C*4of");
  }  
  function btn5click() {
    var btn = document.getElementById("out5");
    if (btn.style.backgroundColor == "") btn.style.backgroundColor = "gray";   
    if (btn.style.backgroundColor == "gray")Socket.send("#C*5on");       
    else Socket.send("#C*5of");
  }
  function btn6click() {
    var btn = document.getElementById("out6");
    if (btn.style.backgroundColor == "") btn.style.backgroundColor = "gray";   
    if (btn.style.backgroundColor == "gray")Socket.send("#C*6on");       
    else Socket.send("#C*6of");
  }

  function setInputFunc(selectedInput){
  if(selectedInput.id == "rbtInputs3_1"
  || selectedInput.id == "rbtInputs3_2"
  || selectedInput.id == "rbtInputs3_3"
  || selectedInput.id == "rbtInputs3"
  ){

   timer1.disabled = true;
   }

  else timer1.disabled = false; 
  }

function SetActualTime()
{
  var t = new Date();
  var mm = t.getMinutes();
  var hh = t.getHours();
  var x = (parseInt(hh)*60)+(parseInt(mm));
  var timeStamp = "&T*"+ x.toString()+ "*";
  Socket.send(timeStamp)
}

  function OutputSelect(selectedOut) {
    // document.getElementById("sel1").innerText = "";
    //  document.getElementById("sel2").innerText = "";
    // document.getElementById("sel3").innerText = "";
    // document.getElementById("sel4").innerText = "";


    if (selectedOut.name == "prepSpin") spinackyLabel = selectedOut;

    //nastaveni kontrolek
    var radioTstat = document.getElementsByName("prepTstat");
    var radioSpin = document.getElementsByName("prepSpin");
    var radioInp1 = document.getElementsByName("prepInp1");
    var radioInp2 = document.getElementsByName("prepInp2");
    var radioInp3 = document.getElementsByName("prepInp3");
    var radioInp4 = document.getElementsByName("prepInp4");
    var labels = document.getElementsByClassName("label");
    var descripts = ["relé 1", "relé 2", "out 1", "out 2", "out 3", "out 4"];

    //karta termostat
    if (selectedOut.name == "prepTstat") {
      termostatLabel = selectedOut;
      //reset
      if (selectedOut.id == "rbtT7") {//reset
        for (ii = 0; ii < radioTstat.length - 1; ii++) {
          if (labels[ii].innerText == "termostat") {
            labels[ii].innerText = "";
            radioSpin[ii].disabled = false;
            radioSpin[ii].labels[0].innerText = descripts[ii];
            radioInp1[ii].disabled = false;
            radioInp1[ii].labels[0].innerText = descripts[ii];
            radioInp2[ii].disabled = false;
            radioInp2[ii].labels[0].innerText = descripts[ii];
            radioInp3[ii].disabled = false;
            radioInp3[ii].labels[0].innerText = descripts[ii];
            radioInp4[ii].disabled = false;
            radioInp4[ii].labels[0].innerText = descripts[ii];
          }
        }
        selectedOut.checked = false;
      }



      for (i = 0; i < radioTstat.length - 1; i++) {
        if (labels[i].innerText == "termostat") labels[i].innerText = "";
        if (radioTstat[i].checked == true) {
          labels[i].innerText = "termostat";
          radioSpin[i].disabled = true;
          radioSpin[i].labels[0].innerText = "";
          radioInp1[i].disabled = true;
          radioInp1[i].labels[0].innerText = "";
          radioInp2[i].disabled = true;
          radioInp2[i].labels[0].innerText = "";
          radioInp3[i].disabled = true;
          radioInp3[i].labels[0].innerText = "";
          radioInp4[i].disabled = true;
          radioInp4[i].labels[0].innerText = "";


        }
        else {
          radioSpin[i].disabled = false;
          radioSpin[i].labels[0].innerText = descripts[i];

          if (labels[i].innerText == "") {
            radioInp1[i].disabled = false;
            radioInp1[i].labels[0].innerText = descripts[i];
            radioInp2[i].disabled = false;
            radioInp2[i].labels[0].innerText = descripts[i];
            radioInp3[i].disabled = false;
            radioInp3[i].labels[0].innerText = descripts[i];
            radioInp4[i].disabled = false;
            radioInp4[i].labels[0].innerText = descripts[i];
          }
          // if (labels[i].innerText != "spinacky") labels[i].innerText = "";
        }

      }

    }

    // karta spinacky
    if (selectedOut.name == "prepSpin") {
      //reset
      if (selectedOut.id == "rbtS7") {//reset
        for (ii = 0; ii < radioSpin.length - 1; ii++) {
          if (labels[ii].innerText == "spínačky") {
            labels[ii].innerText = "";
            radioTstat[ii].disabled = false;
            radioTstat[ii].labels[0].innerText = descripts[ii];
            radioInp1[ii].disabled = false;
            radioInp1[ii].labels[0].innerText = descripts[ii];
            radioInp2[ii].disabled = false;
            radioInp2[ii].labels[0].innerText = descripts[ii];
            radioInp3[ii].disabled = false;
            radioInp3[ii].labels[0].innerText = descripts[ii];
            radioInp4[ii].disabled = false;
            radioInp4[ii].labels[0].innerText = descripts[ii];
          }
        }
        selectedOut.checked = false;
      }
      for (i = 0; i < radioSpin.length - 1; i++) {
        if (labels[i].innerText == "spínačky") labels[i].innerText = "";
        if (radioSpin[i].checked == true) {
          labels[i].innerText = "spínačky";
          radioTstat[i].disabled = true;
          radioTstat[i].labels[0].innerText = "";
          radioInp1[i].disabled = true;
          radioInp1[i].labels[0].innerText = "";
          radioInp2[i].disabled = true;
          radioInp2[i].labels[0].innerText = "";
          radioInp3[i].disabled = true;
          radioInp3[i].labels[0].innerText = "";
          radioInp4[i].disabled = true;
          radioInp4[i].labels[0].innerText = "";
        }
        else {
          radioTstat[i].disabled = false;
          radioTstat[i].labels[0].innerText = descripts[i];
          if (labels[i].innerText == "") {
            radioInp1[i].disabled = false;
            radioInp1[i].labels[0].innerText = descripts[i];
            radioInp2[i].disabled = false;
            radioInp2[i].labels[0].innerText = descripts[i];
            radioInp3[i].disabled = false;
            radioInp3[i].labels[0].innerText = descripts[i];
            radioInp4[i].disabled = false;
            radioInp4[i].labels[0].innerText = descripts[i];
          }
        }
      }
    }

    //karta IN 1
    if (selectedOut.name == "prepInp1") {
      //reset
      if (selectedOut.id == "rbtI1_7") {//reset
        for (ii = 0; ii < radioSpin.length - 1; ii++) {
          if (labels[ii].innerText == "IN 1") {
            labels[ii].innerText = "";
            radioTstat[ii].disabled = false;
            radioTstat[ii].labels[0].innerText = descripts[ii];
            radioSpin[ii].disabled = false;
            radioSpin[ii].labels[0].innerText = descripts[ii];
            radioInp2[ii].disabled = false;
            radioInp2[ii].labels[0].innerText = descripts[ii];
            radioInp3[ii].disabled = false;
            radioInp3[ii].labels[0].innerText = descripts[ii];
            radioInp4[ii].disabled = false;
            radioInp4[ii].labels[0].innerText = descripts[ii];
          }
        }
        selectedOut.checked = false;
      }
      for (i = 0; i < radioInp1.length - 1; i++) {
        if (labels[i].innerText == "IN 1") labels[i].innerText = "";
        if (radioInp1[i].checked == true) {
          labels[i].innerText = "IN 1";
          radioSpin[i].disabled = true;
          radioSpin[i].labels[0].innerText = "";
          radioTstat[i].disabled = true;
          radioTstat[i].labels[0].innerText = "";
          radioInp2[i].disabled = true;
          radioInp2[i].labels[0].innerText = "";
          radioInp3[i].disabled = true;
          radioInp3[i].labels[0].innerText = "";
          radioInp4[i].disabled = true;
          radioInp4[i].labels[0].innerText = "";


        }
        else {
          if (labels[i].innerText == "") {
            radioSpin[i].disabled = false;
            radioSpin[i].labels[0].innerText = descripts[i];
            radioTstat[i].disabled = false;
            radioTstat[i].labels[0].innerText = descripts[i];
            radioInp2[i].disabled = false;
            radioInp2[i].labels[0].innerText = descripts[i];
            radioInp3[i].disabled = false;
            radioInp3[i].labels[0].innerText = descripts[i];
            radioInp4[i].disabled = false;
            radioInp4[i].labels[0].innerText = descripts[i];
          }
        }
      }
    }

    //karta IN 2
    if (selectedOut.name == "prepInp2") {
      //reset
      if (selectedOut.id == "rbtI2_7") {//reset
        for (ii = 0; ii < radioSpin.length - 1; ii++) {
          if (labels[ii].innerText == "IN 2") {
            labels[ii].innerText = "";
            radioTstat[ii].disabled = false;
            radioTstat[ii].labels[0].innerText = descripts[ii];
            radioSpin[ii].disabled = false;
            radioSpin[ii].labels[0].innerText = descripts[ii];
            radioInp1[ii].disabled = false;
            radioInp1[ii].labels[0].innerText = descripts[ii];
            radioInp3[ii].disabled = false;
            radioInp3[ii].labels[0].innerText = descripts[ii];
            radioInp4[ii].disabled = false;
            radioInp4[ii].labels[0].innerText = descripts[ii];
          }
        }
      }
      for (i = 0; i < radioInp2.length - 1; i++) {
        if (labels[i].innerText == "IN 2") labels[i].innerText = "";
        if (radioInp2[i].checked == true) {
          labels[i].innerText = "IN 2";
          radioSpin[i].disabled = true;
          radioSpin[i].labels[0].innerText = "";
          radioTstat[i].disabled = true;
          radioTstat[i].labels[0].innerText = "";
          radioInp1[i].disabled = true;
          radioInp1[i].labels[0].innerText = "";
          radioInp3[i].disabled = true;
          radioInp3[i].labels[0].innerText = "";
          radioInp4[i].disabled = true;
          radioInp4[i].labels[0].innerText = "";
        }
        else {
          // radioSpin[i].disabled = false;
          // radioSpin[i].labels[0].innerText = descripts[i];
          if (labels[i].innerText == "") {
            radioSpin[i].disabled = false;
            radioSpin[i].labels[0].innerText = descripts[i];
            radioTstat[i].disabled = false;
            radioTstat[i].labels[0].innerText = descripts[i];
            radioInp1[i].disabled = false;
            radioInp1[i].labels[0].innerText = descripts[i];
            radioInp3[i].disabled = false;
            radioInp3[i].labels[0].innerText = descripts[i];
            radioInp4[i].disabled = false;
            radioInp4[i].labels[0].innerText = descripts[i];
          }
        }
      }
    }
    //karta IN 3
    if (selectedOut.name == "prepInp3") {
      //reset
      if (selectedOut.id == "rbtI3_7") {//reset
        for (ii = 0; ii < radioSpin.length - 1; ii++) {
          if (labels[ii].innerText == "IN 3") {
            labels[ii].innerText = "";
            radioTstat[ii].disabled = false;
            radioTstat[ii].labels[0].innerText = descripts[ii];
            radioSpin[ii].disabled = false;
            radioSpin[ii].labels[0].innerText = descripts[ii];
            radioInp1[ii].disabled = false;
            radioInp1[ii].labels[0].innerText = descripts[ii];
            radioInp2[ii].disabled = false;
            radioInp2[ii].labels[0].innerText = descripts[ii];
            radioInp4[ii].disabled = false;
            radioInp4[ii].labels[0].innerText = descripts[ii];
          }
        }
        selectedOut.checked = false;
      }
      for (i = 0; i < radioInp3.length - 1; i++) {
        if (labels[i].innerText == "IN 3") labels[i].innerText = "";
        if (radioInp3[i].checked == true) {
          labels[i].innerText = "IN 3";
          radioSpin[i].disabled = true;
          radioSpin[i].labels[0].innerText = "";
          radioTstat[i].disabled = true;
          radioTstat[i].labels[0].innerText = "";
          radioInp1[i].disabled = true;
          radioInp1[i].labels[0].innerText = "";
          radioInp2[i].disabled = true;
          radioInp2[i].labels[0].innerText = "";
          radioInp4[i].disabled = true;
          radioInp4[i].labels[0].innerText = "";


        }
        else {
          if (labels[i].innerText == "") {
            radioSpin[i].disabled = false;
            radioSpin[i].labels[0].innerText = descripts[i];
            radioTstat[i].disabled = false;
            radioTstat[i].labels[0].innerText = descripts[i];
            radioInp1[i].disabled = false;
            radioInp1[i].labels[0].innerText = descripts[i];
            radioInp2[i].disabled = false;
            radioInp2[i].labels[0].innerText = descripts[i];
            radioInp4[i].disabled = false;
            radioInp4[i].labels[0].innerText = descripts[i];
          }
        }
      }
    }
 //   karta IN 4
    if (selectedOut.name == "prepInp4") {
      //reset
      if (selectedOut.id == "rbtI4_7") {//reset
        for (ii = 0; ii < radioSpin.length - 1; ii++) {
          if (labels[ii].innerText == "IN 4") {
            labels[ii].innerText = "";
            radioTstat[ii].disabled = false;
            radioTstat[ii].labels[0].innerText = descripts[ii];
            radioSpin[ii].disabled = false;
            radioSpin[ii].labels[0].innerText = descripts[ii];
            radioInp1[ii].disabled = false;
            radioInp1[ii].labels[0].innerText = descripts[ii];
            radioInp2[ii].disabled = false;
            radioInp2[ii].labels[0].innerText = descripts[ii];
            radioInp3[ii].disabled = false;
            radioInp3[ii].labels[0].innerText = descripts[ii];
          }
        }
        selectedOut.checked = false;
      }
      for (i = 0; i < radioInp4.length - 1; i++) {
        if (labels[i].innerText == "IN 4") labels[i].innerText = "";
        if (radioInp4[i].checked == true) {
          labels[i].innerText = "IN 4";
          radioSpin[i].disabled = true;
          radioSpin[i].labels[0].innerText = "";
          radioTstat[i].disabled = true;
          radioTstat[i].labels[0].innerText = "";
          radioInp1[i].disabled = true;
          radioInp1[i].labels[0].innerText = "";
          radioInp2[i].disabled = true;
          radioInp2[i].labels[0].innerText = "";
          radioInp3[i].disabled = true;
          radioInp3[i].labels[0].innerText = "";
          
        }
        else {
          if (labels[i].innerText == "") {
            radioSpin[i].disabled = false;
            radioSpin[i].labels[0].innerText = descripts[i];
            radioTstat[i].disabled = false;
            radioTstat[i].labels[0].innerText = descripts[i];
            radioInp1[i].disabled = false;
            radioInp1[i].labels[0].innerText = descripts[i];
            radioInp2[i].disabled = false;
            radioInp2[i].labels[0].innerText = descripts[i];
            radioInp3[i].disabled = false;
            radioInp3[i].labels[0].innerText = descripts[i];
          }
       }
      }
    }
  }

  function openSubPage(pageName, elmnt, color) {
    // Hide all elements with class="tabcontent" by default */
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("tabcontent2");
    for (i = 0; i < tabcontent.length; i++) {
      tabcontent[i].style.display = "none";
    }

    // Remove the background color of all tablinks/buttons
    tablinks = document.getElementsByClassName("tablink2");
    for (i = 0; i < tablinks.length; i++) {
      tablinks[i].style.backgroundColor = "";
    }

    // Show the specific tab content
    document.getElementById(pageName).style.display = "block";

    // Add the specific color to the button used to open the tab content
    elmnt.style.backgroundColor = color;
  }

  function openPage(pageName, elmnt, color) {
    // Hide all elements with class="tabcontent" by default */
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
      tabcontent[i].style.display = "none";
    }

    // Remove the background color of all tablinks/buttons
    tablinks = document.getElementsByClassName("tablink");
    for (i = 0; i < tablinks.length; i++) {
      tablinks[i].style.backgroundColor = "";
    }

    // Show the specific tab content
    document.getElementById(pageName).style.display = "block";
    if (pageName == 'nastaveni') openSubPage(inp1div, document.getElementsByName("inp1div"), "lightgreen");
    // Add the specific color to the button used to open the tab content
    elmnt.style.backgroundColor = color;
  }
</script>

<body onload="init()">
  <div class="main">
    <button class="tablink" id="defaultOpen" onclick="openPage('prehled', this, 'blue')">Přehled</button>
    <button class="tablink" onclick="openPage('termostat', this, 'green')">Termostat</button>
    <button class="tablink" onclick="openPage('spinacky', this, 'goldenrod')">Spínačky</button>
    <button class="tablink" onclick="openPage('nastaveni', this, 'lightgreen')">Nastavení</button>
    <div id=prehled class="tabcontent">
      <div>
        <div class="space"></div>
        <table class="mainTable">
          <tbody>
            <div>
            <tr>
              <td>
                <button type="button" class="btn" onclick="btn1click()" id="out1">RELE 1</button>
              </td> 
              <td>
                <button type="button" class="btn  btn-lg" onclick="btn2click()" id="out2">RELE 2</button>
              </td>
              <td> 
                <button type="button" class="btn  btn-lg" onclick="btn3click()" id="out3">OUT 1</button>
              </td>
              <td>
                <button type="button" class="btn  btn-lg" onclick="btn4click()" id="out4">OUT 2</button>
              </td>
              <td>
                <button type="button" class="btn  btn-lg" onclick="btn5click()" id="out5">OUT 3</button>
              </td>
              <td>
                <button type="button" class="btn  btn-lg" onclick="btn6click()" id="out6">OUT 4</button>
               </td>
            </tr>
            <tr>
              <td>
                <p id="sel1" class="label"></p>
              </td>
              <td>
                <p id="sel2" class="label"></p>
              </td>
              <td>
                <p id="sel3" class="label"></p>
              </td>
              <td>
                <p id="sel4" class="label"></p>
              </td>
              <td>
                <p id="sel5" class="label"></p>
              </td>
              <td>
                <p id="sel6" class="label"></p>
              </td>              
            </tr>
        </div>
          </tbody>
        </table>
        <div class="space">
          <div style="margin-top:30px;">
            <span  class="tempLabel" id="tempLabel0" style="width:50px;height:30px;margin-top:20px; ">Teplota </span>
            <span class="tempLabel" id="tempLabel1"  style="width:50px;height:30px;margin-top:20px;">23</span>
            <span class="tempLabel" id=" tempLabel2" style="width:50px;height:30px;margin-top:20px;"> °C</span>
          </div>
          <div style="margin-top:30px;">
            <span class="adcLabel" id="adcLabel1" style="width:50px;height:30px;margin-top:20px; ">Analogový vstup </span>
            <span class="adcLabel" id="adcValLabel" style="width:50px;height:30px;margin-top:20px;">0</span>
          </div>
        </div>
        <div style=" margin: 30px;">
              <button type="button" class="btn  btn-lg" id="in1">IN 1</button>
              <button type="button" class="btn  btn-lg" id="in2">IN 2</button>
              <button type="button" class="btn  btn-lg" id="in3">IN 3</button>
              <button type="button" class="btn  btn-lg" id="in4">IN 4</button>
          </div>
          <div>
            <button type="button"  id="btnSetTime" onclick="SetActualTime()" style="margin-top:50px;" > Nastav aktuální čas</button>
          </div>
        </div>
      </div>
      <!--end-->
      <div id="termostat" class="tabcontent" style="margin:auto" font-size="15px">
        <table style="margin:auto;">
          <thead>
            <tr>
              <th>teplota</th>
              <th class="spc"></th>
              <th>čas</th>
            </tr>

          </thead>
          <tbody>
            <tr>
              <td>
                <input id="t1" class="setTemp" type="number" min="0" max="50" step="0.5" value="20"></input>
              </td>
              <td class="spc"></td>
              <td>
                <input id="tt1" class="setTime" type="time" value="00:00"></input>
              </td>
            </tr>
            <tr>
              <td>
                <input class="setTemp" type="number" min="0" max="50" step="0.5" value="20"></input>
              </td>
              <td class="spc"></td>
              <td>
                <input class="setTime" type="time" value="00:00"></input>
              </td>
            </tr>
            <tr>
              <td>
                <input class="setTemp" type="number" min="0" max="50" step="0.5" value="20"></input>
              </td>
              <td class="spc"></td>
              <td>
                <input class="setTime" type="time" value="00:00"></input>
              </td>
            </tr>
            <tr>
              <td>
                <input class="setTemp" type="number" min="0" max="50" step="0.5" value="20"></input>
              </td>
              <td class="spc"></td>
              <td>
                <input class="setTime" type="time" value="00:00"></input>
              </td>
            </tr>
          </tbody>
        </table>
        <form style="margin-top:50px">
          <fieldset style="border:1;">
            <legend>Výstup termostatu</legend>
            <div style="margin:auto">
              <input type="radio" id="rbtT1" name="prepTstat" onclick="OutputSelect(this)" value="all">
              <label for="rbtT1">relé 1</label>
              <input type="radio" id="rbtT2" name="prepTstat" onclick="OutputSelect(this)" value="false">
              <label for="rbtT2">relé 2</label>
              <input type="radio" id="rbtT3" name="prepTstat" onclick="OutputSelect(this)" value="true">
              <label for="rbtT3">out 1</label>
              <input type="radio" id="rbtT4" name="prepTstat" onclick="OutputSelect(this)" value="true">
              <label for="rbtT4">out 2</label>
              <input type="radio" id="rbtT5" name="prepTstat" onclick="OutputSelect(this)" value="true">
              <label for="rbtT5">out 3</label>
              <input type="radio" id="rbtT6" name="prepTstat" onclick="OutputSelect(this)" value="true">
              <label for="rbtT6">out 4</label>
              <input type="radio" id="rbtT7" name="prepTstat" onclick="OutputSelect(this)" value="true">              
              <label for="rbtT7">RST</label>
            </div>
          </fieldset>
        </form>
          <div style="margin-top:30px;">
            <span  class="tempLabel" id="tempLabel0_1" style="width:50px;height:30px;margin-top:20px; ">Teplota </span>
            <span class="tempLabel" id="tempLabel1_1"  style="width:50px;height:30px;margin-top:20px;">23</span>
            <span class="tempLabel" id=" tempLabel2_1" style="width:50px;height:30px;margin-top:20px;"> °C</span>
          </div>
        <div>
          <button type="submit" onclick="SaveData()">ULOŽIT</button>
          <button type="submit" onclick="getData()">NAČÍST</button>
        </div>
      </div>
      <div id="spinacky" class="tabcontent">
        <table style="margin:auto;">
          <thead>
            <tr>
              <th style="text-align:center">on</th>
              <th class="spc"></th>
              <th style="text-align:center">off</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>
                <input id="sp1on" class="setTimeSpinOn" type="time" value="00:00"></input>
              </td>
              <td class="spc"></td>
              <td>
                <input id="sp1off" class="setTimeSpinOff" type="time" value="00:00"></input>
              </td>
            </tr>
            <tr>
              <td>
                <input id="sp2on" class="setTimeSpinOn" type="time" value="00:00"></input>
              </td>
              <td class="spc"></td>
              <td>
                <input id="sp2off" class="setTimeSpinOff" type="time" value="00:00"></input>
              </td>
            </tr>
            <tr>
              <td>
                <input id="sp3on" class="setTimeSpinOn" type="time" value="00:00"></input>
              </td>
              <td class="spc"></td>
              <td>
                <input id="sp3off" class="setTimeSpinOff" type="time" value="00:00"></input>
              </td>
            </tr>
            <tr>
              <td>
                <input id="sp4on" class="setTimeSpinOn" type="time" value="00:00"></input>
              </td>
              <td class="spc"></td>
              <td>
                <input id="sp4off" class="setTimeSpinOff" type="time" value="00:00"></input>
              </td>
            </tr>
          </tbody>
        </table>
        <form style="margin-top:50px">
          <fieldset style="border:1;">
            <legend>Výstup spínacích hodin</legend>
            <div style="margin:auto">
              <input type="radio" id="rbtS1" name="prepSpin" value="all" onclick="OutputSelect(this)">
              <label for="rbtS1">relé 1</label>
              <input type="radio" id="rbtS2" name="prepSpin" value="false" onclick="OutputSelect(this)">
              <label for="rbtS2">relé 2</label>
              <input type="radio" id="rbtS3" name="prepSpin" value="true" onclick="OutputSelect(this)">
              <label for="rbtS3">out 1</label>
              <input type="radio" id="rbtS4" name="prepSpin" value="true" onclick="OutputSelect(this)">
              <label for="rbtS4">out 2</label>
              <input type="radio" id="rbtS5" name="prepSpin" onclick="OutputSelect(this)" value="true">
              <label for="rbtS5">out 3</label>
              <input type="radio" id="rbtS6" name="prepSpin" onclick="OutputSelect(this)" value="true">
              <label for="rbtS6">out 4</label>
              <input type="radio" id="rbtS7" name="prepSpin" onclick="OutputSelect(this)" value="true">              
              <label for="rbtS7">RST</label
            </div>
          </fieldset>
        </form>
        <div>
          <button type="submit" onclick="SaveData()">ULOŽIT</button>
          <button type="submit" onclick="getData()">NAČÍST</button>
        </div>
      </div>
      <div id="nastaveni" class="tabcontent">
        <button class="tablink2" id="defaultOpen" onclick="openSubPage('inp1div', this, 'lightgreen')">IN 1</button>
        <button class="tablink2" onclick="openSubPage('inp2div', this, 'lightgreen')">IN 2</button>
        <button class="tablink2" onclick="openSubPage('inp3div', this, 'lightgreen')">IN 3</button>
        <button class="tablink2" onclick="openSubPage('inp4div', this, 'lightgreen')">IN 4</button>
        <div id="inp1div" class="tabcontent2">
          <form style="margin-top:50px">
            <fieldset style="border:1;">
              <legend>Funkce vstupu IN1</legend>
              <div style="margin:auto">
                <input type="radio" id="rbtInputs1_1" name="prepInMode1" onclick="setInputFunc(this)" value="all">
                <label for="rbtInputs1_1">trvale</label>
                <input type="radio" id="rbtInputs2_1" name="prepInMode1" onclick="setInputFunc(this)" value="false">
                <label for="rbtInputs2_1">puls</label>
                <input type="radio" id="rbtInputs3_1" name="prepInMode1" onclick="setInputFunc(this)" value="true">
                <label for="rbtInputs3_1">časovač</label>
                <input type="radio" id="rbtInputs4_1" name="prepInMode1" onclick="setInputFunc(this)" value="true">
                <label for="rbtInputs4_1">email</label>
              </div>
            </fieldset>
          </form>
          <form style="margin-top:50px">
            <fieldset style="border:1;">
              <legend>Ovládáný výstup</legend>
              <div style="margin:auto">
                <input type="radio" id="rbtI1_1" name="prepInp1" onclick="OutputSelect(this)" value="all">
                <label for="rbtI1_1">relé 1</label>
                <input type="radio" id="rbtI1_2" name="prepInp1" onclick="OutputSelect(this)" value="false">
                <label for="rbtI1_2">relé 2</label>
                <input type="radio" id="rbtI1_3" name="prepInp1" onclick="OutputSelect(this)" value="true">
                <label for="rbtI1_3">out 1</label>
                <input type="radio" id="rbtI1_4" name="prepInp1" onclick="OutputSelect(this)" value="true">
                <label for="rbtI1_4">out 2</label>
                <input type="radio" id="rbtI1_5" name="prepInp1" onclick="OutputSelect(this)" value="true">
                <label for="rbtI1_5">out 3</label>
                <input type="radio" id="rbtI1_6" name="prepInp1" onclick="OutputSelect(this)" value="true">
                <label for="rbtI1_6">out 4</label>                
                <input type="radio" id="rbtI1_7" name="prepInp1" onclick="OutputSelect(this)" value="true">
                <label for="rbtI1_7">RST</label>
              </div>
            </fieldset>
          </form>
          <div style="margin-top:50px">
            <label for="timer1" class="tmrLabel">čas</label>
            <input type="time" step="1" id="timer1" class="timer" value="00:00:00">
          </div>
        </div>
        <div id="inp2div" class="tabcontent2">
          <form style="margin-top:50px">
            <fieldset style="border:1;">
              <legend>Funkce vstupu IN2</legend>
              <div style="margin:auto">
                <input type="radio" id="rbtInputs1_2" name="prepInMode2" onclick="setInputFunc(this)" value="all">
                <label for="rbtInputs1_2">trvale</label>
                <input type="radio" id="rbtInputs2_2" name="prepInMode2" onclick="setInputFunc(this)" value="false">
                <label for="rbtInputs2_2">puls</label>
                <input type="radio" id="rbtInputs3_2" name="prepInMode2" onclick="setInputFunc(this)" value="true">
                <label for="rbtInputs3_2">časovač</label>
                <input type="radio" id="rbtInputs4_2" name="prepInMode2" onclick="setInputFunc(this)" value="true">
                <label for="rbtInputs4_2">email</label>
              </div>
            </fieldset>
          </form>
          <form style="margin-top:50px">
            <fieldset style="border:1;">
              <legend>Ovládáný výstup</legend>
              <div style="margin:auto">
                <input type="radio" id="rbtI2_1" name="prepInp2" onclick="OutputSelect(this)" value="all">
                <label for="rbtI2_1">relé 1</label>
                <input type="radio" id="rbtI2_2" name="prepInp2" onclick="OutputSelect(this)" value="false">
                <label for="rbtI2_2">relé 2</label>
                <input type="radio" id="rbtI2_3" name="prepInp2" onclick="OutputSelect(this)" value="true">
                <label for="rbtI2_3">out 1</label>
                <input type="radio" id="rbtI2_4" name="prepInp2" onclick="OutputSelect(this)" value="true">
                <label for="rbtI2_4">out 2</label>
                <input type="radio" id="rbtI2_5" name="prepInp2" onclick="OutputSelect(this)" value="true">
                <label for="rbtI2_5">out 3</label>
                <input type="radio" id="rbtI2_6" name="prepInp2" onclick="OutputSelect(this)" value="true">
                <label for="rbtI2_6">out 4</label>                
                <input type="radio" id="rbtI2_7" name="prepInp2" onclick="OutputSelect(this)" value="true">
                <label for="rbtI2_7">RST</label>
              </div>
            </fieldset>
          </form>
          <div style="margin-top:50px">
            <label for="timer1" class="tmrLabel">čas</label>
            <input type="time" step="1" id="timer1" class="timer" value="00:00:00">
          </div>
        </div>
        <div id="inp3div" class="tabcontent2">
          <form style="margin-top:50px">
            <fieldset style="border:1;">
              <legend>Funkce vstupu IN3</legend>
              <div style="margin:auto">
                <input type="radio" id="rbtInputs1" name="prepInMode3" onclick="setInputFunc(this)" value="all">
                <label for="rbtInputs1">trvale</label>
                <input type="radio" id="rbtInputs2" name="prepInMode3" onclick="setInputFunc(this)" value="false">
                <label for="rbtInputs2">puls</label>
                <input type="radio" id="rbtInputs3" name="prepInMode3" onclick="setInputFunc(this)" value="true">
                <label for="rbtInputs3">časovač</label>
                <input type="radio" id="rbtInputs4" name="prepInMode3" onclick="setInputFunc(this)" value="true">
                <label for="rbtInputs4">email</label
              </div>
            </fieldset>
          </form>
          <form style="margin-top:50px">
            <fieldset style="border:1;">
              <legend>Ovládáný výstup</legend>
              <div style="margin:auto">
                <input type="radio" id="rbtI3_1" name="prepInp3" onclick="OutputSelect(this)" value="all">
                <label for="rbtI3_1">relé 1</label>
                <input type="radio" id="rbtI3_2" name="prepInp3" onclick="OutputSelect(this)" value="false">
                <label for="rbtI3_2">relé 2</label>
                <input type="radio" id="rbtI3_3" name="prepInp3" onclick="OutputSelect(this)" value="true">
                <label for="rbtI3_3">out 1</label>
                <input type="radio" id="rbtI3_4" name="prepInp3" onclick="OutputSelect(this)" value="true">
                <label for="rbtI3_4">out 2</label>
                <input type="radio" id="rbtI3_5" name="prepInp3" onclick="OutputSelect(this)" value="true">
                <label for="rbtI3_5">out 3</label>
                <input type="radio" id="rbtI3_6" name="prepInp3" onclick="OutputSelect(this)" value="true">
                <label for="rbtI3_6">out 4</label>                
                <input type="radio" id="rbtI3_7" name="prepInp3" onclick="OutputSelect(this)" value="true">
                <label for="rbtI3_7">RST</label>
              </div>
            </fieldset>
          </form>
          <div style="margin-top:50px">
            <label for="timer1" class="tmrLabel">čas</label>
            <input type="time" step="1" id="timer1" class="timer" value="00:00:00">
          </div>
        </div>
        <div id="inp4div" class="tabcontent2">
          <form style="margin-top:50px">
            <fieldset style="border:1;">
              <legend>Funkce vstupu IN4</legend>
              <div style="margin:auto">
                <input type="radio" id="rbtInputs1_3" name="prepInMode4" onclick="setInputFunc(this)" value="all">
                <label for="rbtInputs1_3">trvale</label>
                <input type="radio" id="rbtInputs2_3" name="prepInMode4" onclick="setInputFunc(this)" value="false">
                <label for="rbtInputs2_3">puls</label>
                <input type="radio" id="rbtInputs3_3" name="prepInMode4" onclick="setInputFunc(this)" value="true">
                <label for="rbtInputs3_3">časovač</label>
                <input type="radio" id="rbtInputs4_3" name="prepInMode4" onclick="setInputFunc(this)" value="true">
                <label for="rbtInputs4_3">email</label
              </div>
            </fieldset>
          </form>
          <form style="margin-top:50px">
            <fieldset style="border:1;">
              <legend>Ovládáný výstup</legend>
              <div style="margin:auto">
                <input type="radio" id="rbtI4_1" name="prepInp4" onclick="OutputSelect(this)" value="all">
                <label for="rbtI4_1">relé 1</label>
                <input type="radio" id="rbtI4_2" name="prepInp4" onclick="OutputSelect(this)" value="false">
                <label for="rbtI4_2">relé 2</label>
                <input type="radio" id="rbtI4_3" name="prepInp4" onclick="OutputSelect(this)" value="true">
                <label for="rbtI4_3">out 1</label>
                <input type="radio" id="rbtI4_4" name="prepInp4" onclick="OutputSelect(this)" value="true">
                <label for="rbtI4_4">out 2</label>
                <input type="radio" id="rbtI4_5" name="prepInp4" onclick="OutputSelect(this)" value="true">
                <label for="rbtI4_5">out 3</label>
                <input type="radio" id="rbtI4_6" name="prepInp4" onclick="OutputSelect(this)" value="true">
                <label for="rbtI4_6">out 4</label>                
                <input type="radio" id="rbtI4_7" name="prepInp4" onclick="OutputSelect(this)" value="true">
                <label for="rbtI4_7">RST</label>
              </div>
            </fieldset>
          </form>
          <div style="margin-top:50px">
            <label for="timer1" class="tmrLabel">čas</label>
            <input type="time" step="1" id="timer1" class="timer" value="00:00:00">
          </div>
        </div>
        <div>
          <button type="submit" onclick="SaveData()">ULOŽIT</button>
          <button type="submit" onclick="getData()">NAČÍST</button>
        </div>        
      </div>
    </div>
</body>

</html>




)=====";

#endif
