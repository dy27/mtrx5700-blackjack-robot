// Button adapted from: https://gumroad.com/d/352b5982db49d8d33a358e94daf71f4d. 
import React from "react";
import "./button.css";

export const Button = ({
    children,
    type,
    onClick,
    buttonStyle,
    buttonSize
  }) => {
    return (
      <button
        onClick={onClick}
        type={type}
      >
        {children}
      </button>
    );
  };