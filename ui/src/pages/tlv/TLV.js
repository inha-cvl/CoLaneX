import React from "react";
import logo from "../../logo.png";
import "./TLV.css";

const TLV = () => {
  return (
    <div className="TLV">
      <header className="TLV-header">
        <img src={logo} className="TLV-logo" alt="logo" />
        <p> Cooperative Lane Change using by V2X </p>
      </header>
    </div>
  );
};

export default TLV;