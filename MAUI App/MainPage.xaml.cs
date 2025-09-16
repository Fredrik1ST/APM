namespace APM_Running_App
{
    public partial class MainPage : ContentPage
    {
        int count = 0;

        public MainPage()
        {
            InitializeComponent();

        }

        private void StartRun_Clicked(object sender, EventArgs e)
        {
            
            Navigation.PushAsync(new SteadyPacePage());

        }

        private void ViewResults_Clicked(object sender, EventArgs e)
        {

            Navigation.PushAsync(new PreviousResultsPage());

        }
    }

}
