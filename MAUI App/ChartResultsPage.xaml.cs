using Microcharts;
using SkiaSharp;

// The page ChartResultsPage is inspired from 
// Gerald Versluis  7. aug. 2023, accessed [29.04.2024]
// url: https://www.youtube.com/watch?v=yMG8oPIuMig

namespace APM_Running_App
{
    public partial class ChartResultsPage : ContentPage
    {
        ChartEntry[] enteries = new[]
        {

            new ChartEntry(212)
            {
                Label = "Test1",
                ValueLabel = "112",
                Color = SKColor.Parse("#2c3e50"),
            }
        };

        public ChartResultsPage()
        {
            InitializeComponent();
            ChartView.Chart = new LineChart()
            {
                Entries = enteries,
                LabelColor = SKColor.Parse("#2c3e50"),
            };
        }

    }

}
