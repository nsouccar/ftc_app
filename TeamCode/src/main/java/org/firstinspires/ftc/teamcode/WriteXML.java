package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.lang.*;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;

public class WriteXML {
    static int numOfMoves;
    static ArrayList<Double> moveInfo;



    public WriteXML( int numberOfMoves, ArrayList<Double> moveInfoArrayList ){
        this.numOfMoves = numberOfMoves;
        this.moveInfo = moveInfoArrayList;
        System.out.println(this.moveInfo);

    }


    public  void makeXML() {



        DocumentBuilderFactory icFactory = DocumentBuilderFactory.newInstance();
        DocumentBuilder icBuilder;
        List listOfMoves;


        try {

            icBuilder = icFactory.newDocumentBuilder();
            Document doc = icBuilder.newDocument();
            Element mainRootElement = doc.createElementNS("Linear", "Moves");
            doc.appendChild(mainRootElement);

            // append child elements to root element



            for(int i = 0; i < (numOfMoves * 4); i+= 4){
                Node move = createMove(doc, Integer.toString(i/4), moveInfo.get(i + 0),  moveInfo.get(i+ 1), moveInfo.get(i+ 2),moveInfo.get(i+ 3), i);
                mainRootElement.appendChild(move);

            }

            // output DOM XML to console

            TransformerFactory transformerFactory = TransformerFactory.newInstance();
            Transformer transformer = transformerFactory.newTransformer();
            transformer.setOutputProperty(OutputKeys.INDENT, "yes");
            DOMSource source = new DOMSource(doc);
            StreamResult result = new StreamResult(new File("/sdcard/FIRST"));
            transformer.transform(source, result);
            System.out.println("\nXML DOM Created Successfully..");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static Node createMove(Document doc, String id, Double leftInches, Double rightInches, Double timeout, Double angle, int moveNumber ) {
        Element move = doc.createElement("Move");
        move.setAttribute("id", id);
        move.appendChild(getCompanyElements(doc, move, "RightInches", rightInches));
        move.appendChild(getCompanyElements(doc, move, "LeftInches", leftInches));
        move.appendChild(getCompanyElements(doc, move, "Timeout", timeout));
        move.appendChild(getCompanyElements(doc, move,"Angle", angle ));
        return move;
    }

    // utility method to create text node
    private static Node getCompanyElements(Document doc, Element element, String name, double value) {
        Element node = doc.createElement(name);
        node.appendChild(doc.createTextNode(Double.toString(value)));
        return node;
    }
}
