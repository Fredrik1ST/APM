namespace APM_Running_App
{
    public partial class AppShell : Shell
    {
        public AppShell()
        {
            InitializeComponent();

            Routing.RegisterRoute("MainPage", typeof(MainPage));
            Routing.RegisterRoute("PreviousResults", typeof(PreviousResultsPage));
            Routing.RegisterRoute("ChartResults", typeof(ChartResultsPage));
            Routing.RegisterRoute("Info", typeof(InfoPage));
            Routing.RegisterRoute("Settings", typeof(SettingsPage));

            Routing.RegisterRoute("SteadyPacePage", typeof(SteadyPacePage));
            Routing.RegisterRoute("UserDefinedPage", typeof(UserDefinedPage));
            Routing.RegisterRoute("PatternElitePage", typeof(PatternElitePage));
            Routing.RegisterRoute("IntervalPage", typeof(IntervalPage));
            Routing.RegisterRoute("InfoPage", typeof(InfoPage));
            Routing.RegisterRoute("SettingsPage", typeof(SettingsPage));


        }
        protected override bool OnBackButtonPressed()
        {
            Task<bool> answer = DisplayAlert("Question?", "Do you want to exit?", "Yes", "No");
            answer.ContinueWith(task =>
            {
                if (task.Result)
                {
                    Application.Current.Quit();
                }
            });
            return true;
        }
        private void HelpCommand(object sender, EventArgs e)
        {

            DisplayAlert("Info", "This APP is made for the APM. To start the APM, go to training page and choose workout type. Then press Start Run", "Return");

        }

    }
}
