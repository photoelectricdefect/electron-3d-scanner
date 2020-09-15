void plot2D(std::vector<Eigen::Vector3f> pts) {
    TApplication app("Graph", 0, 0);
    TCanvas *c1 = new TCanvas("c1","plane points",0,0,600,400);   
   TGraph2D *dt = new TGraph2D(pts.size());
    
    for(int i = 0; i < pts.size(); i++) {
        std::cout << "plot pt: " << std::endl << pts[i] << std::endl;
    }
    
    int i = 0;
    auto compare = [i](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
        return abs(a(i)) < abs(b(i));
    };
   auto max_x = std::max_element(pts.begin(), pts.end(), compare);
   i++;
   auto max_y = std::max_element(pts.begin(), pts.end(), compare);
   
    std::cout << "max_x: " << std::endl << *max_x << std::endl;
    std::cout << "max_y: " << std::endl << *max_y << std::endl;
   
   dt->SetNpy((*max_y)(1));
   dt->SetNpx((*max_x)(0));
   Int_t k = 0;

    for(int i = 0; i < pts.size(); i++) {
        dt->SetPoint(k, pts[i](0), pts[i](1), pts[i](2));
        k++;
    }

    gStyle->SetPalette(1);
    dt->SetMarkerStyle(20);
    dt->Draw("pcol");
    app.Run();
}