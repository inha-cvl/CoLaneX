import { makeStyles } from "@material-ui/core";

export default makeStyles((theme) => ({
  map: {
    height: "100vh",
  },
  whole: {
    alignItems:"stretch"
  },
  basic50: {
    position: "relative",
    display: "flex",
    marginTop: "0.5rem",
    marginRight: "0.45rem",
    height: "10rem",
    width: "100%",
    fontFamily: "Rajdhani, sans-serif",
    fontSize: "8rem",
    borderRadius: "4px",
    boxShadow: "5px 5px 7px #05060e, -5px -5px 7px #131a3a",
    backgroundColor: "#0c1024",
    color: "#fff",
    "&:hover": {
      borderRadius: "4px",
      boxShadow: "5px 5px 7px #05060e, -5px -5px 7px #131a3a",
      backgroundColor: "#0c1024",
      color: "#fff",
    },
  },
  purple50: {
    position: "relative",
    display: "flex",
    marginTop: "0.5rem",
    marginRight: "0.45rem",
    height: "10rem",
    width: "100%",
    fontFamily: "Rajdhani, sans-serif",
    fontSize: "8rem",
    width: "100%",
    borderRadius: "4px",
    boxShadow: "5px 5px 7px #05060e, -5px -5px 7px #131a3a",
    backgroundColor: "#6569d7",
    color: "#fff",
    "&:hover": {
      borderRadius: "4px",
      boxShadow: "5px 5px 7px #05060e, -5px -5px 7px #131a3a",
      backgroundColor: "#6569d7",
      color: "#fff",
    },
  },
  basic_card: {
    backgroundColor: "#0c1024",
    color: "#fff",
    height: "25rem",
    fontSize: "5rem",
    display: "flex",
    boxShadow: "5px 5px 7px #05060e, -5px -5px 7px #131a3a",
    verticalAlign: "middle",
    justifyContent: "center",
    marginTop: "2rem",
    alignItems: "center",
  },
}));