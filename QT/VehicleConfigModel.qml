import QtQuick 2.0

ListModel {
    id: vehicleConfigModel
    ListElement{
        name: "engineRPM"
        displayText: "Engine RPM"
        value: 5000
        unit: ""
    }
    ListElement{
        name: "tyreDiameter"
        displayText: "Tyre Diameter"
        value: 680
        unit: "mm"
    }
    ListElement{
        name: "firstGear"
        displayText: "First Gear Ratio"
        value: 2.7
        unit: ""
    }
    ListElement{
        name: "secondGear"
        displayText: "Second Gear Ratio"
        value: 2.1
        unit: ""
    }
    ListElement{
        name: "thirdGear"
        displayText: "Third Gear Ratio"
        value: 1.5
        unit: ""
    }
    ListElement{
        name: "fourthGear"
        displayText: "Fourth Gear Ratio"
        value: 1
        unit: ""
    }
    ListElement{
        name: "fifthGear"
        displayText: "Fifth Gear Ratio"
        value: 0.85
        unit: ""
    }
    ListElement{
        name: "sixthGear"
        displayText: "Sixth Gear Ratio"
        value: 0.6
        unit: ""
    }


}
