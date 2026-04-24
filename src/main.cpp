#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

#include <STEPControl_Writer.hxx>
#include <IFSelect_ReturnStatus.hxx>

#include <iostream>

int main()
{
    // 创建一个长宽高分别为 100、50、30 的 box
    Standard_Real dx = 100.0;
    Standard_Real dy = 50.0;
    Standard_Real dz = 30.0;

    TopoDS_Shape box = BRepPrimAPI_MakeBox(dx, dy, dz).Shape();

    // 导出为 STEP 文件
    STEPControl_Writer writer;
    IFSelect_ReturnStatus status = writer.Transfer(box, STEPControl_AsIs);

    if (status != IFSelect_RetDone)
    {
        std::cerr << "STEP transfer failed." << std::endl;
        return 1;
    }

    status = writer.Write("box.step");

    if (status != IFSelect_RetDone)
    {
        std::cerr << "STEP write failed." << std::endl;
        return 1;
    }

    std::cout << "Box created and exported to box.step" << std::endl;

    return 0;
}